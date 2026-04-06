#ifndef evaluators_gazebo_h
#define evaluators_gazebo_h


#include "cstring"
#include "cstdio"
#include "cstdlib"
#include "thread"
#include "unistd.h"

#include "EvalData.h"
#include "GazeboEnv.h"
#include "ActionWrappers.h"
#include "TPG.h"
#include "filesystem"
#include "misc.h"


#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

#ifndef TPG_TRACE
#define TPG_TRACE 0
#endif

#if TPG_TRACE
inline void trace_log(const std::string& msg) {
  static std::mutex trace_mtx;
  std::lock_guard<std::mutex> lk(trace_mtx);
  std::cerr << msg << std::endl;
}
#define TRACE(MSG) do { \
  std::ostringstream _oss; \
  _oss << MSG; \
  trace_log(_oss.str()); \
} while (0)
#else
#define TRACE(MSG) do {} while (0)
#endif




inline void EvalGazebo(TPG& tpg, EvalData& eval_data) {

  GazeboEnv* task = dynamic_cast<GazeboEnv*>(eval_data.task);

  task->Reset(tpg.rngs_[AUX_SEED]);

  eval_data.n_prediction = 0;
  eval_data.obs = new state (task->GetObsSize());
  eval_data.obs->Set(task->GetObsVec(eval_data.partially_observable));



  static std::shared_ptr<std::ofstream> log = nullptr;
  static std::shared_ptr<std::ofstream> sim_obs = nullptr;
  static std::shared_ptr<std::ofstream> reward = nullptr;
  static std::shared_ptr<std::ofstream> traj = nullptr;
  static std::shared_ptr<std::ofstream> world = nullptr;
  float x_prev = task->GetRobotX(), x;
  float y_prev = task->GetRobotY(), y;
  float total_distance = 0;

  float pixel_dist = 0;

  if(tpg.GetParam<int>("replay") && eval_data.episode == 0) {
    if(tpg.GetParam<int>("animate")){
      std::system("ign gazebo -g -r &");
    }

    std::filesystem::path out_dir = std::filesystem::current_path()/"logs/best_agent"; 
    std::filesystem::create_directories(out_dir);

    log = std::make_shared<std::ofstream>(out_dir / "best_agent_stats.csv" );
    *log << "episode, reached_goal, steps, path_length, optimal_path_length, fitness\n";

    traj = std::make_shared<std::ofstream>(out_dir / "trajectory.csv" );
    *traj << "episode, step, robot_x, robot_y, robot_yaw\n";

    world = std::make_shared<std::ofstream>(out_dir / "world_layout.csv" );
    *world << "episode, goal_x, goal_y, obstacle_x, obstacle_y, obstacle_width, obstacle_depth\n";

    /* for (size_t i = 0; i < v.size(); ++i) { */
    /*   *sim_obs << ",obs_" << i; */
    /* } */

    /* *sim_obs << "\n"; */



    reward = std::make_shared<std::ofstream>(out_dir/ "reward.csv");
    *reward << "step, reward, center pixel dist\n";

  }

  if (tpg.GetParam<int>("replay")) {

    *world << eval_data.episode << ","
           << task->GetGoalX() << ","
           << task->GetGoalY() << ","
           << task->GetObstacleX() << ","
           << task->GetObstacleY() << ","
           << task->GetObstacleWidth() << ","
           << task->GetObstacleDepth() << "\n";
  }

  /* int counts[3] = {0,0,0}; */

  double fitness = 0.0;


  while (!task->Terminal()) {

    tpg.GetAction(eval_data);  


    auto ctrl = WrapDiscreteActionGazebo(eval_data);


    std::vector<double> action = (ctrl == 0) ? std::vector<double>{0.46,0.0} :
				 (ctrl == 1) ? std::vector<double>{0.05, 1.20} : 
				 (ctrl == 2) ? std::vector<double>{0.05, -1.20} :
				 std::vector<double>{0.0, 0.0};



    TaskEnv::Results r = task->SimStep(action);



    //Calculating fitness over episode for boxplot
    fitness += r.r1;

    eval_data.stats_double[REWARD1_IDX] += r.r1;  
    eval_data.AccumulateStepData();

    eval_data.obs->Set(task->GetObsVec(eval_data.partially_observable));





    if(tpg.GetParam<int>("replay")) {      

	x = task->GetRobotX(); y = task->GetRobotY();

	total_distance += std::sqrt(std::pow(x-x_prev,2) + std::pow(y-y_prev,2));

	x_prev = x; y_prev = y;

	*traj << eval_data.episode << ","
	      << task->step_ << ","
	      << task->GetRobotX() << ","
	      << task->GetRobotY() << ","
	      << task->GetRobotYaw() << "\n";

    }

  }

  /* std::cout << "ctrl counts: " << counts[0] << " " << counts[1] << " " << counts[2] << "\n"; */

  if(tpg.GetParam<int>("replay")) {

    int goals_reached = 0;
    double optimal_path = 0; 

    optimal_path = task->GetOptimalPathLength()-0.35;

    if(task->step_ < tpg.GetParam<int>("gz_max_timestep") ) { goals_reached++; }

    *log << eval_data.episode << "," << goals_reached << "," << task->step_ << "," << total_distance << "," << optimal_path << "," << fitness << "\n";

    if(eval_data.episode == tpg.GetParam<int>("gz_n_eval_test") - 1) {

	log->close();
	reward->close();

	if (traj) traj->close();
	if (world) world->close();
    }

  }




  delete eval_data.obs;
}




#endif 
