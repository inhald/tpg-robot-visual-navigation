#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>


#include "../src/environments/gazebo/GazeboEnv.h"
#include "../src/environments/gazebo/TurtleBot4_Nav.h"
#include "ignition/msgs/pose_v.pb.h"


#include <random>
#include <cstdlib>
#include <iostream>
#include <TPG.h>
#include <instruction.h>
#include <chrono>
#include <cmath>


using Catch::Approx;

std::unordered_map<std::string, std::any> createDefaultParams() {

    const char* tpg_env = std::getenv("TPG");
    if (tpg_env == nullptr) {
        throw std::runtime_error("TPG environment variable is not set.");
    }

    setenv("IGN_PARTITION", "turtlebot4", 1);

    
    
    std::unordered_map<std::string, std::any> params;
    params["gz_obs_type"] = std::string("lidar");
    params["gz_model_path"] = std::string("$TPG/datasets/gazebo_models/turtlebot4.sdf");
    params["gz_world_path"] = std::string("$TPG/datasets/gazebo_models/maze.sdf");
    params["gz_object_path"] = std::string("$TPG/datasets/gazebo_models/models/cereal_box/model.sdf");
    params["gz_max_timestep"] = 10000;
    /* params["cv_depth_hi"] = 120; */
    /* params["cv_depth_hf"] = 200; */ 
    /* params["cv_depth_wi"] = 100; */
    /* params["cv_depth_wf"] = 300; */

    params["cv_depth_height"] = 15;
    params["cv_depth_width"] = 60;

    params["cv_rgb_height"] = 60;
    params["cv_rgb_width"] = 80;

    params["gz_n_eval_train"] = 200;
    params["gz_n_eval_validation"] = 0;
    params["gz_n_eval_test"] = 1;
    params["gz_max_timestep"] = 200;
    params["gz_reward_control_weight"] = 0.0;

    params["replay"] = 0;

    return params;

}

TEST_CASE("Gazebo Simulation", "[init]"){
  std::unordered_map<std::string, std::any> params = createDefaultParams(); 
  TurtleBot4_Nav env(params);

  std::cout << "Terminating simulation" << std::endl;
}

TEST_CASE("Gazebo Simulation - SetState", "[SetState]"){
  std::unordered_map<std::string, std::any> params = createDefaultParams(); 
  TurtleBot4_Nav env(params);

  /* REQUIRE(env.WaitForFirstPose()); */

  std::vector<float> endpose = {1.5f, 1.5f};

  env.set_state(endpose);
  /* env.do_simulation(); */

  REQUIRE(abs(1.5 - env.GetRobotX()) < 0.01);
  REQUIRE(abs(1.5 - env.GetRobotY()) < 0.01);

}
