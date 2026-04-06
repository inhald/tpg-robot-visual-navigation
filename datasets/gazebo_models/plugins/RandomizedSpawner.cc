// SimpleRandomizedSpawnerECS.cpp

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <optional>
#include <random>
#include <string>
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

    if (_sdf && _sdf->HasElement("robot_name"))
      this->robotName = _sdf->Get<std::string>("robot_name");

    if (_sdf && _sdf->HasElement("base_seed"))
      this->baseSeed = _sdf->Get<uint64_t>("base_seed");

    if (_sdf && _sdf->HasElement("min_x"))
      this->minX = _sdf->Get<double>("min_x");
    if (_sdf && _sdf->HasElement("max_x"))
      this->maxX = _sdf->Get<double>("max_x");
    if (_sdf && _sdf->HasElement("min_y"))
      this->minY = _sdf->Get<double>("min_y");
    if (_sdf && _sdf->HasElement("max_y"))
      this->maxY = _sdf->Get<double>("max_y");

    if (_sdf && _sdf->HasElement("sample_about_robot"))
      this->sampleAboutRobot = _sdf->Get<bool>("sample_about_robot");

    if (_sdf && _sdf->HasElement("sample_center_x"))
      this->sampleCenterX = _sdf->Get<double>("sample_center_x");
    if (_sdf && _sdf->HasElement("sample_center_y"))
      this->sampleCenterY = _sdf->Get<double>("sample_center_y");

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

    if (_sdf && _sdf->HasElement("target_half_width"))
      this->targetHalfW = std::max(0.0, _sdf->Get<double>("target_half_width"));
    if (_sdf && _sdf->HasElement("target_half_depth"))
      this->targetHalfD = std::max(0.0, _sdf->Get<double>("target_half_depth"));
    if (_sdf && _sdf->HasElement("target_aabb_height"))
      this->targetAabbH = std::max(0.1, _sdf->Get<double>("target_aabb_height"));

    if (_sdf && _sdf->HasElement("obs_size_x"))
      this->obsSizeX = std::max(0.0, _sdf->Get<double>("obs_size_x"));
    if (_sdf && _sdf->HasElement("obs_size_y"))
      this->obsSizeY = std::max(0.0, _sdf->Get<double>("obs_size_y"));
    if (_sdf && _sdf->HasElement("obs_size_z"))
      this->obsSizeZ = std::max(0.1, _sdf->Get<double>("obs_size_z"));

    if (_sdf && _sdf->HasElement("extra_separation"))
      this->extraSep = std::max(0.0, _sdf->Get<double>("extra_separation"));

    if (_sdf && _sdf->HasElement("wall_margin"))
      this->wallMargin = std::max(0.0, _sdf->Get<double>("wall_margin"));

    if (_sdf && _sdf->HasElement("max_goal_tries"))
      this->maxGoalTries = std::max(1, _sdf->Get<int>("max_goal_tries"));
    if (_sdf && _sdf->HasElement("max_obs_tries"))
      this->maxObsTries = std::max(1, _sdf->Get<int>("max_obs_tries"));

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
        Request req;
        try {
          for (const auto &kv : msg.header().data()) {
            if (kv.key() == "spawn_seed" && kv.value_size() > 0)
              req.seed = std::stoull(kv.value(0));
            else if (kv.key() == "req_id" && kv.value_size() > 0)
              req.req_id = std::stoull(kv.value(0));
          }
        } catch (...) {
          std::cerr << "[SimpleRandomizedSpawnerECS] Bad request header\n";
          return;
        }

        if (req.req_id == 0) {
          std::cerr << "[SimpleRandomizedSpawnerECS] Missing req_id\n";
          return;
        }

        {
          std::lock_guard<std::mutex> lk(this->reqMtx);
          this->pendingReq = req;
          this->hasPendingReq = true;
        }
      }
    );
  }

  void PostUpdate(const UpdateInfo &,
                  const EntityComponentManager &_ecm) override
  {
    Request req;
    {
      std::lock_guard<std::mutex> lk(this->reqMtx);
      if (!this->hasPendingReq)
        return;
      req = this->pendingReq;
      this->hasPendingReq = false;
    }

    if (this->robotEntity == kNullEntity)
      this->FindRobotEntity(_ecm);

    double rx = 0.0, ry = 0.0, yaw = 0.0;
    if (!this->FetchRobotPoseYaw(_ecm, rx, ry, yaw)) {
      this->robotEntity = kNullEntity;
      this->FindRobotEntity(_ecm);
      if (!this->FetchRobotPoseYaw(_ecm, rx, ry, yaw)) {
        this->PublishFallback(req.req_id, 0.0, 0.0);
        return;
      }
    }

    const double cx = sampleAboutRobot ? rx : sampleCenterX;
    const double cy = sampleAboutRobot ? ry : sampleCenterY;

    std::mt19937_64 rng(mix64(baseSeed ^ req.seed));

    const double obsHalfW = 0.5 * obsSizeX;
    const double obsHalfD = 0.5 * obsSizeY;

    const double goalClear = std::max(targetHalfW, targetHalfD) + extraSep + wallMargin;
    const double obsClear  = std::max(obsHalfW, obsHalfD) + extraSep + wallMargin;

    const double goalRMaxEff = std::min(goalRMax, MaxRadiusSafeFromWalls(cx, cy, goalClear));
    const double obsRMaxEff  = std::min(obsRMax,  MaxRadiusSafeFromWalls(cx, cy, obsClear));

    if (goalRMaxEff <= goalRMin || obsRMaxEff <= obsRMin) {
      PublishFallback(req.req_id, rx, ry);
      return;
    }

    for (int g = 0; g < maxGoalTries; ++g) {
      const double g_theta = SampleAngle(rng);
      const double g_rho   = SampleRadiusArea(rng, goalRMin, goalRMaxEff);

      const double gx = cx + g_rho * std::cos(g_theta);
      const double gy = cy + g_rho * std::sin(g_theta);

      if (!InBounds(gx, gy, targetHalfW, targetHalfD))
        continue;
      if (!FitsAndFree(_ecm, gx, gy, targetHalfW, targetHalfD, targetAabbH))
        continue;

      auto goalAABB = Inflate(MakeAABB(gx, gy, targetHalfW, targetHalfD, targetAabbH), extraSep);

      for (int o = 0; o < maxObsTries; ++o) {
        const double o_theta = SampleAngle(rng);
        const double o_rho   = SampleRadiusArea(rng, obsRMin, obsRMaxEff);

        const double ox = cx + o_rho * std::cos(o_theta);
        const double oy = cy + o_rho * std::sin(o_theta);

        if (!InBounds(ox, oy, obsHalfW, obsHalfD))
          continue;
        if (!FitsAndFree(_ecm, ox, oy, obsHalfW, obsHalfD, obsSizeZ))
          continue;

        auto obsAABB = Inflate(MakeAABB(ox, oy, obsHalfW, obsHalfD, obsSizeZ), extraSep);
        if (goalAABB.Intersects(obsAABB))
          continue;

        PublishPair(req.req_id, gx, gy, ox, oy);
        return;
      }
    }

    PublishFallback(req.req_id, rx, ry);
  }

private:
  struct Request {
    uint64_t seed{0};
    uint64_t req_id{0};
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

  static double SampleAngle(std::mt19937_64 &rng)
  {
    std::uniform_real_distribution<double> u(-M_PI, M_PI);
    return u(rng);
  }

  static double SampleRadiusArea(std::mt19937_64 &rng, double rmin, double rmax)
  {
    std::uniform_real_distribution<double> u(0.0, 1.0);
    const double x = u(rng);
    return std::sqrt(rmin * rmin + x * (rmax * rmax - rmin * rmin));
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
    if (robotEntity != kNullEntity)
      return;

    _ecm.Each<components::Name, components::Pose>(
      [&](const Entity &e,
          const components::Name *n,
          const components::Pose *) -> bool
      {
        if (n && n->Data() == robotName) {
          robotEntity = e;
          return false;
        }
        return true;
      });
  }

  bool FetchRobotPoseYaw(const EntityComponentManager &_ecm,
                         double &rx, double &ry, double &yaw)
  {
    if (robotEntity == kNullEntity)
      return false;

    auto *poseComp = _ecm.Component<components::Pose>(robotEntity);
    auto *nameComp = _ecm.Component<components::Name>(robotEntity);

    if (!poseComp || !nameComp || nameComp->Data() != robotName) {
      robotEntity = kNullEntity;
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

    if (nm.find("target") != std::string::npos)
      return true;
    if (nm.find("obstacle") != std::string::npos)
      return true;
    if (nm.find("cereal") != std::string::npos)
      return true;
    if (nm.find("box") != std::string::npos)
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
                   double height) const
  {
    if (!InBounds(X, Y, halfW, halfD))
      return false;

    ignition::math::AxisAlignedBox cand(
      {X - halfW, Y - halfD, 0.0},
      {X + halfW, Y + halfD, height});

    bool collides = false;

    _ecm.Each<components::AxisAlignedBox, components::Name>(
      [&](const Entity &,
          const components::AxisAlignedBox *bx,
          const components::Name *nm) -> bool
      {
        if (!bx)
          return true;

        if (nm && NameIgnored(nm->Data()))
          return true;

        if (cand.Intersects(bx->Data())) {
          collides = true;
          return false;
        }

        return true;
      });

    return !collides;
  }

  void PublishPair(uint64_t req_id, double gx, double gy, double ox, double oy)
  {
    msgs::Pose_V out;

    auto *h = out.mutable_header();
    auto *d = h->add_data();
    d->set_key("req_id");
    d->add_value(std::to_string(req_id));

    auto *pGoal = out.add_pose();
    pGoal->set_name("target");
    pGoal->mutable_position()->set_x(gx);
    pGoal->mutable_position()->set_y(gy);
    pGoal->mutable_position()->set_z(0.0);
    pGoal->mutable_orientation()->set_w(1.0);

    auto *pObs = out.add_pose();
    pObs->set_name("obstacle");
    pObs->mutable_position()->set_x(ox);
    pObs->mutable_position()->set_y(oy);
    pObs->mutable_position()->set_z(0.0);
    pObs->mutable_orientation()->set_w(1.0);

    responsePub.Publish(out);
  }

  void PublishFallback(uint64_t req_id, double rx, double ry)
  {
    const double obsHalfW = 0.5 * obsSizeX;
    const double obsHalfD = 0.5 * obsSizeY;

    const double gx = std::clamp(rx + 0.75, minX + targetHalfW, maxX - targetHalfW);
    const double gy = std::clamp(ry + 0.00, minY + targetHalfD, maxY - targetHalfD);
    const double ox = std::clamp(rx - 0.75, minX + obsHalfW, maxX - obsHalfW);
    const double oy = std::clamp(ry + 0.00, minY + obsHalfD, maxY - obsHalfD);

    PublishPair(req_id, gx, gy, ox, oy);
  }

private:
  Entity worldEntity{kNullEntity};
  Entity robotEntity{kNullEntity};

  std::string robotName{"turtlebot4"};
  uint64_t baseSeed{42};

  double minX{-5.0}, maxX{5.0}, minY{-5.0}, maxY{5.0};

  bool sampleAboutRobot{false};
  double sampleCenterX{0.0};
  double sampleCenterY{0.0};

  double goalRMin{0.8}, goalRMax{1.25};
  double obsRMin{0.6}, obsRMax{1.25};

  double targetHalfW{0.10}, targetHalfD{0.10}, targetAabbH{0.40};
  double obsSizeX{0.50}, obsSizeY{0.50}, obsSizeZ{0.75};

  double extraSep{0.05};
  double wallMargin{0.05};

  int maxGoalTries{300};
  int maxObsTries{300};

  std::vector<std::string> ignoreNameContains;

  std::mutex reqMtx;
  Request pendingReq;
  bool hasPendingReq{false};

  transport::Node node{MakeOpts()};
  transport::Node::Publisher responsePub;
};

} // namespace ignition::gazebo

IGNITION_ADD_PLUGIN(
  ignition::gazebo::RandomizedSpawnerECS,
  ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPostUpdate)
