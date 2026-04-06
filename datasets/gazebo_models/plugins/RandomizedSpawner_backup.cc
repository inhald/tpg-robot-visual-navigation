// RandomizedSpawnerECS.cpp
//
// Rewritten to make request handoff thread-safe.
// Main changes:
// 1) /random_pose_req callback writes a whole request under a mutex.
// 2) PostUpdate consumes that whole request under the same mutex.
// 3) last-success fallback remains mutex-protected.
// 4) Robot entity is re-found if stale.
// 5) Same transport API: /random_pose_req -> /random_pose_res
//

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>

#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/transport/Node.hh>

#include <ignition/plugin/Register.hh>

namespace ignition::gazebo {

class RandomizedSpawnerECS final
  : public System
  , public ISystemConfigure
  , public ISystemPostUpdate
{
public:
  void Configure(const Entity &,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override
  {
    this->worldEntity = ignition::gazebo::worldEntity(_ecm);

    // ---------------- SDF parameters ----------------
    if (_sdf && _sdf->HasElement("robot_name"))
      this->robotName = _sdf->Get<std::string>("robot_name");

    if (_sdf && _sdf->HasElement("base_seed"))
      this->baseSeed = _sdf->Get<uint64_t>("base_seed");

    if (_sdf && _sdf->HasElement("require_in_fov"))
      this->requireInFov = _sdf->Get<bool>("require_in_fov");

    if (_sdf && _sdf->HasElement("hfov_deg"))
      this->hfovRad = _sdf->Get<double>("hfov_deg") * M_PI / 180.0;

    if (_sdf && _sdf->HasElement("min_x"))
      this->minX = _sdf->Get<double>("min_x");
    if (_sdf && _sdf->HasElement("max_x"))
      this->maxX = _sdf->Get<double>("max_x");
    if (_sdf && _sdf->HasElement("min_y"))
      this->minY = _sdf->Get<double>("min_y");
    if (_sdf && _sdf->HasElement("max_y"))
      this->maxY = _sdf->Get<double>("max_y");

    if (_sdf && _sdf->HasElement("half_width"))
      this->halfWidth = std::max(0.0, _sdf->Get<double>("half_width"));
    if (_sdf && _sdf->HasElement("half_depth"))
      this->halfDepth = std::max(0.0, _sdf->Get<double>("half_depth"));
    if (_sdf && _sdf->HasElement("aabb_height"))
      this->aabbHeight = std::max(0.1, _sdf->Get<double>("aabb_height"));

    if (_sdf && _sdf->HasElement("goal_r_min"))
      this->goalRMin = std::max(0.0, _sdf->Get<double>("goal_r_min"));
    if (_sdf && _sdf->HasElement("goal_r_max"))
      this->goalRMax = std::max(0.0, _sdf->Get<double>("goal_r_max"));
    if (goalRMax < goalRMin)
      std::swap(goalRMin, goalRMax);

    if (_sdf && _sdf->HasElement("obs_r_min"))
      this->obsRMin = std::max(0.0, _sdf->Get<double>("obs_r_min"));
    if (_sdf && _sdf->HasElement("obs_r_max"))
      this->obsRMax = std::max(0.0, _sdf->Get<double>("obs_r_max"));
    if (obsRMax < obsRMin)
      std::swap(obsRMin, obsRMax);

    if (_sdf && _sdf->HasElement("obs_size_x"))
      this->obsSizeX = std::max(0.0, _sdf->Get<double>("obs_size_x"));
    if (_sdf && _sdf->HasElement("obs_size_y"))
      this->obsSizeY = std::max(0.0, _sdf->Get<double>("obs_size_y"));
    if (_sdf && _sdf->HasElement("obs_size_z"))
      this->obsSizeZ = std::max(0.0, _sdf->Get<double>("obs_size_z"));

    if (_sdf && _sdf->HasElement("extra_separation"))
      this->extraSep = std::max(0.0, _sdf->Get<double>("extra_separation"));

    if (_sdf && _sdf->HasElement("target_half_width"))
      this->targetHalfW = std::max(0.0, _sdf->Get<double>("target_half_width"));
    if (_sdf && _sdf->HasElement("target_half_depth"))
      this->targetHalfD = std::max(0.0, _sdf->Get<double>("target_half_depth"));
    if (_sdf && _sdf->HasElement("target_aabb_height"))
      this->targetAabbH = std::max(0.1, _sdf->Get<double>("target_aabb_height"));

    if (_sdf && _sdf->HasElement("max_goal_tries"))
      this->maxGoalTries = std::max(1, _sdf->Get<int>("max_goal_tries"));
    if (_sdf && _sdf->HasElement("max_obs_tries"))
      this->maxObsTries = std::max(1, _sdf->Get<int>("max_obs_tries"));

    if (_sdf && _sdf->HasElement("wall_margin"))
      this->wallMargin = std::max(0.0, _sdf->Get<double>("wall_margin"));

    if (_sdf && _sdf->HasElement("relax_on_fail"))
      this->relaxOnFail = _sdf->Get<bool>("relax_on_fail");

    if (_sdf && _sdf->HasElement("relax_factor"))
      this->relaxFactor = std::clamp(_sdf->Get<double>("relax_factor"), 0.0, 1.0);

    if (_sdf && _sdf->HasElement("use_last_success_fallback"))
      this->useLastSuccessFallback = _sdf->Get<bool>("use_last_success_fallback");

    // Optional: if true, sample around robot pose. Otherwise sample around fixed center.
    if (_sdf && _sdf->HasElement("sample_about_robot"))
      this->sampleAboutRobot = _sdf->Get<bool>("sample_about_robot");

    if (_sdf && _sdf->HasElement("sample_center_x"))
      this->sampleCenterX = _sdf->Get<double>("sample_center_x");

    if (_sdf && _sdf->HasElement("sample_center_y"))
      this->sampleCenterY = _sdf->Get<double>("sample_center_y");

    // Ignore list
    if (_sdf && _sdf->HasElement("ignore_name_contains")) {
      auto sdf_nc = std::const_pointer_cast<sdf::Element>(_sdf);
      auto el = sdf_nc->GetElement("ignore_name_contains");
      while (el) {
        ignoreNameContains.push_back(el->Get<std::string>());
        el = el->GetNextElement("ignore_name_contains");
      }
    }

    if (ignoreNameContains.empty()) {
      ignoreNameContains = {
        "ground_plane", "floor", "arena", "world", "sun", "light"
      };
    }

    this->FindRobotEntity(_ecm);

    this->responsePub = this->node.Advertise<msgs::Pose_V>("/random_pose_res");

    this->node.Subscribe<ignition::msgs::Boolean>(
      "/random_pose_req",
      [this](const ignition::msgs::Boolean &msg)
      {
        PendingRequest req;
	bool valid = true;

        {
          std::lock_guard<std::mutex> lock(this->pendingReqMtx);

          // Start from previous values so omitted header fields don't zero them.
          req.round = this->pendingReq.round;
          req.ep = this->pendingReq.ep;
          req.seed = 0;
	  req.req_id = 0;
          req.pending = true;

          for (const auto &kv : msg.header().data()) {
            if (kv.key() == "round" && kv.value_size() > 0)
              req.round = std::stoi(kv.value(0));
            else if (kv.key() == "ep" && kv.value_size() > 0)
              req.ep = std::stoi(kv.value(0));
            else if (kv.key() == "spawn_seed" && kv.value_size() > 0)
              req.seed = std::stoull(kv.value(0));
	    else if (kv.key() == "req_id" && kv.value_size() > 0)
	      req.req_id = std::stoull(kv.value(0));
	    
          }

	  if (req.req_id == 0) {
	    valid = false;
	  }
	  else {
	    this->pendingReq = req;
	  }

        }
	if(!valid) {
	  std::cerr << "Request invalid" << std::endl;

	}
        this->callbackCount.fetch_add(1, std::memory_order_relaxed);
      }
    );
  }

  void PostUpdate(const UpdateInfo &,
                  const EntityComponentManager &_ecm) override
  {
    PendingRequest req;
    {
      std::lock_guard<std::mutex> lock(this->pendingReqMtx);
      if (!this->pendingReq.pending)
        return;

      req = this->pendingReq;
      this->pendingReq.pending = false;
    }

    this->consumeCount.fetch_add(1, std::memory_order_relaxed);

    if (robotEntity == kNullEntity)
      FindRobotEntity(_ecm);

    double rx, ry, yaw;
    if (!FetchRobotPoseYaw(_ecm, rx, ry, yaw)) {
      this->robotEntity = kNullEntity;
      FindRobotEntity(_ecm);
      if (!FetchRobotPoseYaw(_ecm, rx, ry, yaw)) {
        std::cout << "[RandomizedSpawnerECS] Robot pose unavailable\n";
        return;
      }
    }

    std::mt19937_64 rng(mix64(baseSeed ^ req.seed));

    const double obsHalfW = 0.5 * obsSizeX;
    const double obsHalfD = 0.5 * obsSizeY;
    const double obsH = std::max(0.1, obsSizeZ);

    const double goalClear = std::max(targetHalfW, targetHalfD) + extraSep + wallMargin;
    const double obsClear  = std::max(obsHalfW, obsHalfD) + extraSep + wallMargin;

    const double cx = sampleAboutRobot ? rx : sampleCenterX;
    const double cy = sampleAboutRobot ? ry : sampleCenterY;

    const double goalRMaxEff = std::min(goalRMax, MaxRadiusSafeFromWalls(cx, cy, goalClear));
    const double obsRMaxEff  = std::min(obsRMax,  MaxRadiusSafeFromWalls(cx, cy, obsClear));

    if (goalRMaxEff <= goalRMin || obsRMaxEff <= obsRMin) {
      FailWithFallback(req.req_id, "sampling center infeasible for given radii", rx, ry);
      return;
    }

    struct RejectStats {
      uint64_t g_oob = 0, g_coll = 0, g_ok = 0;
      uint64_t o_oob = 0, o_coll = 0, o_goalint = 0, o_ok = 0;
      std::unordered_map<std::string, uint64_t> hit;
    } st;

    auto tryPlaceWithSep = [&](double sep) -> bool {
      for (int g = 0; g < maxGoalTries; ++g) {
        const double g_theta = SampleAngle(rng);
        const double g_rho   = SampleRadiusArea(rng, goalRMin, goalRMaxEff);

        const double gx = cx + g_rho * std::cos(g_theta);
        const double gy = cy + g_rho * std::sin(g_theta);

        std::string gHit;
        if (!InBounds(gx, gy, targetHalfW, targetHalfD)) {
          st.g_oob++;
          continue;
        }
        if (!FitsAndFree(_ecm, gx, gy, targetHalfW, targetHalfD, targetAabbH, &gHit)) {
          st.g_coll++;
          if (!gHit.empty())
            st.hit[gHit]++;
          continue;
        }
        st.g_ok++;

        auto goalAABB = Inflate(MakeAABB(gx, gy, targetHalfW, targetHalfD, targetAabbH), sep);

        for (int o = 0; o < maxObsTries; ++o) {
          const double o_theta = SampleAngle(rng);
          const double o_rho   = SampleRadiusArea(rng, obsRMin, obsRMaxEff);

          const double ox = cx + o_rho * std::cos(o_theta);
          const double oy = cy + o_rho * std::sin(o_theta);

          std::string oHit;
          if (!InBounds(ox, oy, obsHalfW, obsHalfD)) {
            st.o_oob++;
            continue;
          }
          if (!FitsAndFree(_ecm, ox, oy, obsHalfW, obsHalfD, obsH, &oHit)) {
            st.o_coll++;
            if (!oHit.empty())
              st.hit[oHit]++;
            continue;
          }
          st.o_ok++;

          auto obsAABB = Inflate(MakeAABB(ox, oy, obsHalfW, obsHalfD, obsH), sep);

          if (goalAABB.Intersects(obsAABB)) {
            st.o_goalint++;
            continue;
          }

          PublishPair(req.req_id, gx, gy, ox, oy);
          CacheLastSuccess(gx, gy, ox, oy);
          return true;
        }
      }
      return false;
    };

    if (tryPlaceWithSep(extraSep))
      return;

    if (relaxOnFail && extraSep > 0.0) {
      const double relaxedSep = extraSep * relaxFactor;
      if (tryPlaceWithSep(relaxedSep))
        return;
    }

    std::cout << "[RandomizedSpawnerECS] Failed. "
              << "goal(oob=" << st.g_oob
              << ", coll=" << st.g_coll
              << ", ok=" << st.g_ok << ") "
              << "obs(oob=" << st.o_oob
              << ", coll=" << st.o_coll
              << ", goalInt=" << st.o_goalint
              << ", ok=" << st.o_ok << ")";

    if (!st.hit.empty()) {
      std::vector<std::pair<std::string, uint64_t>> v(st.hit.begin(), st.hit.end());
      std::sort(v.begin(), v.end(),
                [](const auto &a, const auto &b) { return a.second > b.second; });

      std::cout << " top_hit=[";
      for (size_t i = 0; i < v.size() && i < 3; ++i) {
        std::cout << v[i].first << ":" << v[i].second;
        if (i + 1 < v.size() && i + 1 < 3)
          std::cout << ", ";
      }
      std::cout << "]";
    }

    std::cout << " callback_count=" << callbackCount.load(std::memory_order_relaxed)
              << " consume_count=" << consumeCount.load(std::memory_order_relaxed)
              << "\n";

    FailWithFallback(req.req_id, "rejection sampling exhausted", rx, ry);
  }

private:
  struct PendingRequest {
    int round{0};
    int ep{0};
    uint64_t seed{0};
    uint64_t req_id{0};
    bool pending{false};
  };

  struct LastSuccess {
    double gx{0.0};
    double gy{0.0};
    double ox{0.0};
    double oy{0.0};
  };

  static transport::NodeOptions MakeOpts()
  {
    transport::NodeOptions opts;
    if (const char *p = std::getenv("IGN_PARTITION")) {
      if (*p)
        opts.SetPartition(p);
    }
    return opts;
  }

  static uint64_t mix64(uint64_t x)
  {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
  }

  static double SampleRadiusArea(std::mt19937_64 &rng, double rmin, double rmax)
  {
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const double u = u01(rng);
    return std::sqrt(rmin * rmin + u * (rmax * rmax - rmin * rmin));
  }

  static double SampleAngle(std::mt19937_64 &rng)
  {
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    return -M_PI + 2.0 * M_PI * u01(rng);
  }

  static ignition::math::AxisAlignedBox Inflate(ignition::math::AxisAlignedBox b, double d)
  {
    b.Min().X(b.Min().X() - d);
    b.Min().Y(b.Min().Y() - d);
    b.Max().X(b.Max().X() + d);
    b.Max().Y(b.Max().Y() + d);
    return b;
  }

  ignition::math::AxisAlignedBox MakeAABB(double x, double y,
                                          double halfW, double halfD,
                                          double height) const
  {
    return ignition::math::AxisAlignedBox(
      {x - halfW, y - halfD, 0.0},
      {x + halfW, y + halfD, height}
    );
  }

  bool InBounds(double x, double y, double halfW, double halfD) const
  {
    if (x < minX + halfW || x > maxX - halfW)
      return false;
    if (y < minY + halfD || y > maxY - halfD)
      return false;
    return true;
  }

  double MaxRadiusSafeFromWalls(double x, double y, double margin) const
  {
    const double dxp = (maxX - x) - margin;
    const double dxm = (x - minX) - margin;
    const double dyp = (maxY - y) - margin;
    const double dym = (y - minY) - margin;
    return std::max(0.0, std::min({dxp, dxm, dyp, dym}));
  }

  void FindRobotEntity(const EntityComponentManager &_ecm)
  {
    if (this->robotEntity != kNullEntity)
      return;

    _ecm.Each<components::Name, components::Pose>(
      [&](const Entity &e,
          const components::Name *n,
          const components::Pose *) -> bool
      {
        if (n && n->Data() == this->robotName) {
          this->robotEntity = e;
          return false;
        }
        return true;
      });
  }

  bool FetchRobotPoseYaw(const EntityComponentManager &_ecm,
                         double &rx, double &ry, double &yaw)
  {
    if (this->robotEntity == kNullEntity)
      return false;

    auto *poseComp = _ecm.Component<components::Pose>(this->robotEntity);
    auto *nameComp = _ecm.Component<components::Name>(this->robotEntity);

    if (!poseComp || !nameComp || nameComp->Data() != this->robotName) {
      this->robotEntity = kNullEntity;
      return false;
    }

    const auto &p = poseComp->Data();
    rx = p.Pos().X();
    ry = p.Pos().Y();
    yaw = p.Rot().Yaw();
    return true;
  }

  bool NameIgnored(const std::string &nm) const
  {
    if (nm == robotName)
      return true;

    // Ignore movable spawned entities already placed by your env
    if (nm.find("cereal") != std::string::npos)
      return true;
    if (nm.find("box") != std::string::npos)
      return true;
    if (nm.find("target") != std::string::npos)
      return true;
    if (nm.find("obstacle") != std::string::npos)
      return true;

    for (const auto &s : ignoreNameContains) {
      if (!s.empty() && nm.find(s) != std::string::npos)
        return true;
    }
    return false;
  }

  bool FitsAndFree(const EntityComponentManager &_ecm,
                   double X, double Y,
                   double halfW, double halfD,
                   double height,
                   std::string *hitName = nullptr) const
  {
    if (!InBounds(X, Y, halfW, halfD))
      return false;

    ignition::math::AxisAlignedBox cand(
      {X - halfW, Y - halfD, 0.0},
      {X + halfW, Y + halfD, height});

    bool collides = false;
    std::string localHit;

    _ecm.Each<components::AxisAlignedBox, components::Name>(
      [&](const Entity &,
          const components::AxisAlignedBox *bx,
          const components::Name *nm) -> bool
      {
        if (!bx)
          return true;

        if (nm) {
          const auto &name = nm->Data();
          if (NameIgnored(name))
            return true;
        }

        if (cand.Intersects(bx->Data())) {
          collides = true;
          if (nm)
            localHit = nm->Data();
          return false;
        }

        return true;
      });

    if (collides && hitName)
      *hitName = localHit;

    return !collides;
  }

  void PublishPair(uint64_t req_id, double gx, double gy, double ox, double oy)
  {
    msgs::Pose_V out;

    auto *h = out.mutable_header();
    auto *d = h->add_data();
    d->set_key("req_id");
    d->add_value(std::to_string(req_id));

    auto *p_goal = out.add_pose();
    p_goal->set_name("target");
    p_goal->mutable_position()->set_x(gx);
    p_goal->mutable_position()->set_y(gy);
    p_goal->mutable_position()->set_z(0.0);
    p_goal->mutable_orientation()->set_w(1.0);

    auto *p_obs = out.add_pose();
    p_obs->set_name("obstacle");
    p_obs->mutable_position()->set_x(ox);
    p_obs->mutable_position()->set_y(oy);
    p_obs->mutable_position()->set_z(0.0);
    p_obs->mutable_orientation()->set_w(1.0);

    responsePub.Publish(out);
  }

  void CacheLastSuccess(double gx, double gy, double ox, double oy)
  {
    {
      std::lock_guard<std::mutex> lock(lastSuccessMtx);
      lastSuccess.gx = gx;
      lastSuccess.gy = gy;
      lastSuccess.ox = ox;
      lastSuccess.oy = oy;
    }
    haveLastSuccess.store(true, std::memory_order_release);
  }

  void FailWithFallback(uint64_t req_id, const std::string &why, double rx, double ry)
  {
    if (useLastSuccessFallback && haveLastSuccess.load(std::memory_order_acquire)) {
      double gx, gy, ox, oy;
      {
        std::lock_guard<std::mutex> lock(lastSuccessMtx);
        gx = lastSuccess.gx;
        gy = lastSuccess.gy;
        ox = lastSuccess.ox;
        oy = lastSuccess.oy;
      }

      std::cout << "[RandomizedSpawnerECS] Fallback(last_success): " << why << "\n";
      PublishPair(req_id, gx, gy, ox, oy);
      return;
    }

    const double gx = std::clamp(rx + 0.75, minX + targetHalfW, maxX - targetHalfW);
    const double gy = std::clamp(ry + 0.00, minY + targetHalfD, maxY - targetHalfD);
    const double ox = std::clamp(rx - 0.75, minX + 0.5 * obsSizeX, maxX - 0.5 * obsSizeX);
    const double oy = std::clamp(ry + 0.00, minY + 0.5 * obsSizeY, maxY - 0.5 * obsSizeY);

    std::cout << "[RandomizedSpawnerECS] Fallback(safe_default): " << why << "\n";
    PublishPair(req_id, gx, gy, ox, oy);
  }

private:
  Entity worldEntity{kNullEntity};
  std::string robotName{"turtlebot4"};
  Entity robotEntity{kNullEntity};

  double minX{-5.0}, maxX{5.0}, minY{-5.0}, maxY{5.0};
  double halfWidth{0.5}, halfDepth{0.5};
  double aabbHeight{2.0};

  uint64_t baseSeed{42};

  bool requireInFov{false};
  double hfovRad{M_PI / 2.0};

  // Sampling behaviour
  bool sampleAboutRobot{false};
  double sampleCenterX{0.0};
  double sampleCenterY{0.0};

  // Thread-safe pending request handoff
  std::mutex pendingReqMtx;
  PendingRequest pendingReq;

  // Transport
  transport::Node node{MakeOpts()};
  transport::Node::Publisher responsePub;

  // Sampling annulus parameters
  double goalRMin{0.8}, goalRMax{1.25};
  double obsRMin{0.6}, obsRMax{1.25};

  // Target/Obstacle sizes
  double targetHalfW{0.10}, targetHalfD{0.10}, targetAabbH{0.40};
  double obsSizeX{0.50}, obsSizeY{0.50}, obsSizeZ{0.75};

  // Separation + robustness
  double extraSep{0.05};
  double wallMargin{0.05};
  bool relaxOnFail{true};
  double relaxFactor{0.5};
  bool useLastSuccessFallback{true};

  int maxGoalTries{300};
  int maxObsTries{300};

  std::vector<std::string> ignoreNameContains;

  // Last success cache
  std::mutex lastSuccessMtx;
  LastSuccess lastSuccess;
  std::atomic<bool> haveLastSuccess{false};

  // Diagnostics
  std::atomic<uint64_t> callbackCount{0};
  std::atomic<uint64_t> consumeCount{0};
};

} // namespace ignition::gazebo

IGNITION_ADD_PLUGIN(ignition::gazebo::RandomizedSpawnerECS,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPostUpdate)
