// PathLengthPlugin.cc  (Ignition Gazebo API)
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>

#include <ignition/transport/Node.hh>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/common/Console.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/empty.pb.h>

#include <queue>
#include <vector>
#include <mutex>
#include <cmath>
#include <limits>
#include <array>
#include <string>
#include <algorithm>

using namespace ignition;

namespace
{
struct Grid {
  double res{0.05};
  double originX{0.0};
  double originY{0.0};
  int w{0};
  int h{0};
  std::vector<uint8_t> occ; // 0 free, 1 occupied
  inline bool Inside(int i,int j) const { return i>=0 && j>=0 && i<w && j<h; }
  inline int Index(int i,int j) const { return j*w + i; }
};

static const std::array<std::pair<int,int>,8> kDirs = {{
  {+1,0},{-1,0},{0,+1},{0,-1},{+1,+1},{+1,-1},{-1,+1},{-1,-1}
}};
}

class PathLengthPlugin
  : public gazebo::System,
    public gazebo::ISystemConfigure,
    public gazebo::ISystemPreUpdate // use PreUpdate to rebuild grid (mutable ECM)
{
public:
  void Configure(const gazebo::Entity &,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gazebo::EntityComponentManager &ecm,
                 gazebo::EventManager &) override
  {
    if (sdf && sdf->HasElement("resolution"))
      this->resolution = std::max(1e-3, sdf->Get<double>("resolution"));
    if (sdf && sdf->HasElement("padding"))
      this->padding = std::max(0.0, sdf->Get<double>("padding"));
    if (sdf && sdf->HasElement("goal_topic"))
      this->goalTopic = sdf->Get<std::string>("goal_topic");
    if (sdf && sdf->HasElement("result_topic"))
      this->resultTopic = sdf->Get<std::string>("result_topic");
    if (sdf && sdf->HasElement("diag"))
      this->diag = sdf->Get<bool>("diag");

    this->node.Advertise<msgs::Double>(this->resultTopic);
    this->node.Subscribe(this->goalTopic, &PathLengthPlugin::OnGoal, this);
    this->node.Advertise(this->rebuildSrv, &PathLengthPlugin::OnRebuild, this);

    igndbg << "[PathLengthPlugin] res=" << this->resolution
           << " pad=" << this->padding
           << " diag=" << this->diag
           << " goal=" << this->goalTopic
           << " out=" << this->resultTopic << std::endl;

    // Initial build
    this->RebuildGrid(ecm);


    this->responsePub = this->node.Advertise<msgs::Double>(this->resultTopic);




  }

  void PreUpdate(const gazebo::UpdateInfo &,
                 gazebo::EntityComponentManager &ecm) override
  {
    if (this->rebuildRequested.exchange(false))
      this->RebuildGrid(ecm);
  }

private:
  void RebuildGrid(gazebo::EntityComponentManager &ecm)
  {
    std::lock_guard<std::mutex> lk(this->mtx);

    std::vector<math::AxisAlignedBox> aabbs;
    aabbs.reserve(256);

    bool any = false;
    ecm.Each<gazebo::components::Collision,
             gazebo::components::AxisAlignedBox>(
      [&](const gazebo::Entity &,
          const gazebo::components::Collision *,
          const gazebo::components::AxisAlignedBox *bb)->bool
      {
        if (bb) { any = true; aabbs.push_back(bb->Data()); }
        return true;
      });

    if (!any)
      ignwarn << "[PathLengthPlugin] No AxisAlignedBox components found. "
              << "Did you load the BoundingBox system?\n";

    math::AxisAlignedBox bounds;
    if (!aabbs.empty()) {
      bounds = aabbs.front();
      for (size_t i=1; i<aabbs.size(); ++i) bounds += aabbs[i];
    } else {
      bounds = math::AxisAlignedBox({-10,-10,-1},{+10,+10,+1});
    }

    bounds.Min().X() -= this->padding; bounds.Min().Y() -= this->padding;
    bounds.Max().X() += this->padding; bounds.Max().Y() += this->padding;

    Grid g;
    g.res = this->resolution;
    g.originX = bounds.Min().X();
    g.originY = bounds.Min().Y();

    const double spanX = bounds.Size().X();
    const double spanY = bounds.Size().Y();
    g.w = std::max(1, (int)std::ceil(spanX / g.res));
    g.h = std::max(1, (int)std::ceil(spanY / g.res));
    g.occ.assign((size_t)g.w * g.h, 0);

    for (const auto &b : aabbs)
    {
      const double minx=b.Min().X(), miny=b.Min().Y();
      const double maxx=b.Max().X(), maxy=b.Max().Y();

      int i0 = (int)std::floor((minx - g.originX) / g.res);
      int j0 = (int)std::floor((miny - g.originY) / g.res);
      int i1 = (int)std::ceil ((maxx - g.originX) / g.res);
      int j1 = (int)std::ceil ((maxy - g.originY) / g.res);

      i0 = std::clamp(i0, 0, g.w-1);
      j0 = std::clamp(j0, 0, g.h-1);
      i1 = std::clamp(i1, 0, g.w);
      j1 = std::clamp(j1, 0, g.h);

      for (int j=j0; j<j1; ++j)
        for (int i=i0; i<i1; ++i)
          g.occ[g.Index(i,j)] = 1;
    }

    this->grid = std::move(g);
    this->gridReady = true;

    std::cout << "[PathLengthPlugin] Grid: " << grid.w << "x" << grid.h
           << " @ " << grid.res << " m, origin=("
           << grid.originX << "," << grid.originY << ")\n";
  }

  void OnGoal(const msgs::Pose &msg)
  {
    const math::Vector2d goal(msg.position().x(), msg.position().y());
    double L = std::numeric_limits<double>::quiet_NaN();
    {
      std::lock_guard<std::mutex> lk(this->mtx);
      if (this->gridReady)
        L = this->AStarLength({0,0}, goal);
      else
        ignwarn << "[PathLengthPlugin] Grid not ready; ignoring goal.\n";
    }

    msgs::Double out; out.set_data(L);
    this->responsePub.Publish(out);
    /* this->node.Publish(this->resultTopic, out); */
    std::cout << "[PathLengthPlugin] goal=(" << goal.X() << "," << goal.Y()
           << ") length=" << L << " m\n";
  }

  bool OnRebuild(const msgs::Empty &, msgs::Boolean &rep)
  {
    this->rebuildRequested = true;
    rep.set_data(true);
    return true;
  }

  double AStarLength(const math::Vector2d &startW,
                     const math::Vector2d &goalW)
  {
    const Grid &g = this->grid;
    auto toIJ = [&](const math::Vector2d &p, int &i, int &j)->bool{
      i = (int)std::floor((p.X() - g.originX)/g.res);
      j = (int)std::floor((p.Y() - g.originY)/g.res);
      return g.Inside(i,j);
    };

    int si,sj, gi,gj;
    if (!toIJ(startW,si,sj) || !toIJ(goalW,gi,gj)) return std::numeric_limits<double>::quiet_NaN();
    if (g.occ[g.Index(si,sj)] || g.occ[g.Index(gi,gj)]) return std::numeric_limits<double>::quiet_NaN();

    // neighbor set
    struct N { int di,dj; double cost; };
    std::vector<N> nbrs; nbrs.reserve(this->diag?8:4);
    for (auto d : kDirs) {
      const bool diag = (std::abs(d.first)+std::abs(d.second)==2);
      if (diag && !this->diag) continue;
      nbrs.push_back({d.first, d.second, g.res*(diag?std::sqrt(2.0):1.0)});
    }

    auto h = [&](int i,int j){
      const double dx=(i-gi)*g.res, dy=(j-gj)*g.res;
      return std::sqrt(dx*dx+dy*dy);
    };

    struct QN{int i,j; double f;};
    struct Cmp{bool operator()(const QN&a,const QN&b)const{return a.f>b.f;}};
    std::priority_queue<QN,std::vector<QN>,Cmp> open;

    const int Ncell = g.w*g.h;
    std::vector<double> gscore(Ncell, std::numeric_limits<double>::infinity());
    std::vector<int> parent(Ncell, -1);
    std::vector<char> closed(Ncell, 0);

    const auto idx = [&](int i,int j){return g.Index(i,j);};

    const int sidx = idx(si,sj), goalidx = idx(gi,gj);
    gscore[sidx]=0.0; open.push({si,sj,h(si,sj)});

    while(!open.empty()){
      auto cur=open.top(); open.pop();
      const int cidx = idx(cur.i,cur.j);
      if (closed[cidx]) continue;
      closed[cidx]=1;
      if (cidx==goalidx){
        double L=0;
        int k=goalidx;
        while(k!=sidx && k>=0){
          int p=parent[k]; if(p<0) break;
          int i1=k%g.w, j1=k/g.w, i0=p%g.w, j0=p/g.w;
          const bool d=(std::abs(i1-i0)+std::abs(j1-j0)==2);
          L += g.res*(d?std::sqrt(2.0):1.0);
          k=p;
        }
        return L;
      }
      for(const auto &nb:nbrs){
        int ni=cur.i+nb.di, nj=cur.j+nb.dj;
        if(!g.Inside(ni,nj)) continue;
        int nidx = idx(ni,nj);
        if (g.occ[nidx]) continue;
        double tentative=gscore[cidx]+nb.cost;
        if (tentative<gscore[nidx]){
          gscore[nidx]=tentative;
          parent[nidx]=cidx;
          open.push({ni,nj,tentative+h(ni,nj)});
        }
      }
    }
    return std::numeric_limits<double>::quiet_NaN(); // no path
  }

private:
  // params
  double resolution{0.05};
  double padding{1.0};
  bool diag{true};
  std::string goalTopic{"/path_length/goal"};
  std::string resultTopic{"/path_length/optimal"};
  std::string rebuildSrv{"/path_length/rebuild"};

  // state
  transport::Node node;
  transport::Node::Publisher responsePub;


  Grid grid;
  bool gridReady{false};
  std::mutex mtx;
  std::atomic<bool> rebuildRequested{false};
};

// ----- Registration (Ignition) -----
#include <ignition/plugin/Register.hh>
IGNITION_ADD_PLUGIN(PathLengthPlugin,
                    ignition::gazebo::System,
                    PathLengthPlugin::ISystemConfigure,
                    PathLengthPlugin::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(PathLengthPlugin, "path_length_plugin")


