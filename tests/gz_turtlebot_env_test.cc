#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "ignition/msgs/twist.pb.h"	
#include "ignition/msgs/laserscan.pb.h"
#include "ignition/transport.hh"


#include "../src/environments/gazebo/GazeboEnv.h"
#include "../src/environments/gazebo/TurtleBot4_Nav.h"
#include <random>
#include <TPG.h>
#include <instruction.h>


using Catch::Approx;

// Helper function to create default parameters
std::unordered_map<std::string, std::any> createDefaultParams() {

    const char* tpg_env = std::getenv("TPG");
    if (tpg_env == nullptr) {
        throw std::runtime_error("TPG environment variable is not set.");
    }
    
    setenv("IGN_PARTITION", "turtlebot4", 1);

    std::unordered_map<std::string, std::any> params;
    params["gz_obs_type"] = std::string("depth");
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

TEST_CASE("Turtlebot4 Env Init", "[init]"){
  std::unordered_map<std::string, std::any> params = createDefaultParams(); 

  INFO("Initializing environment");
  TurtleBot4_Nav env(params);

  REQUIRE(env.getParam() == "depth");


}


TEST_CASE("Turtlebot4 Terminal Condition #1", "[terminal]"){
  std::unordered_map<std::string, std::any> params = createDefaultParams(); 
  TurtleBot4_Nav env(params);

  INFO("Testing Terminal Step Count");
  env.SetSteps(2000); 

  REQUIRE(env.Terminal() == true);
}

/* TEST_CASE("Turtlebot4 Terminal Condition #2", "[terminal]") { */
/*   ignition::transport::Node node; */
/*   std::unordered_map<std::string, std::any> params = createDefaultParams(); */ 
/*   TurtleBot4_Nav env(params); */

/*   /1* REQUIRE(env.isRunning()); *1/ */

/*   /1* REQUIRE(env.WaitForFirstScan());   // We find out that it takes about 3-4s of initialization *1/ */ 
/* 				     // latency for the lidar to get receive a measurement in sim */

/*   /1* ignition::msgs::Twist collision_msg; *1/ */ 
/*   /1* collision_msg.mutable_linear()->set_x(0.46); *1/ */
/*   /1* collision_msg.mutable_linear()->set_y(0); *1/ */
/*   /1* collision_msg.mutable_linear()->set_z(0); *1/ */

/*   /1* collision_msg.mutable_angular()->set_x(0); *1/ */
/*   /1* collision_msg.mutable_angular()->set_y(0); *1/ */
/*   /1* collision_msg.mutable_angular()->set_z(0); *1/ */

/*   std::vector<double> ctrl_1 = {0.46, 0.0}; */


/*   std::string topic = "/model/turtlebot4/cmd_vel"; */
/*   auto pub = node.Advertise<ignition::msgs::Twist>(topic); */

/*   /1* while (!pub.HasConnections()) *1/ */
/*   /1*   std::this_thread::sleep_for(std::chrono::milliseconds(5)); *1/ */

/*   int i = 0; */

/*   while (i < 300 && env.Terminal() == false) { */
/*       env.do_simulation(ctrl_1); */
/*       i+=1; */
/*   } */

/*   std::vector<double> ctrl_2 = {0.0,0.0}; */

/*   /1* ignition::msgs::Twist msg; *1/ */ 
/*   /1* msg.mutable_linear()->set_x(0.0); *1/ */
/*   /1* msg.mutable_angular()->set_z(0.0); *1/ */
/*   /1* pub.Publish(msg); *1/ */

/*   env.do_simulation(ctrl_2); */

/*   REQUIRE(env.Terminal() == true); */

/* } */

TEST_CASE("TurtleBot4 Reset", "[reset]") {

  ignition::transport::Node node;
  std::unordered_map<std::string, std::any> params = createDefaultParams(); 
  TurtleBot4_Nav env(params);

  std::vector<double> ctrl_1 = {0.3, 0.00};


  std::string topic = "/model/turtlebot4/cmd_vel";
  auto pub = node.Advertise<ignition::msgs::Twist>(topic);


  env.do_simulation(ctrl_1);


  std::vector<double> ctrl_2 = {0.0, 0.0};

  env.do_simulation(ctrl_2);
  
  std::random_device rd;
  std::mt19937 rng(rd());

  env.Reset(rng);

  REQUIRE(abs(env.GetRobotX() - 0.0) < 0.01);
  REQUIRE(abs(env.GetRobotY() - 0.0) < 0.01);


  REQUIRE(env.Terminal() == false);
  REQUIRE(env.GetCollisionStatus() == false);


}


/* TEST_CASE("TurtleBot4 Reset 2", "[reset]") { */

/*   ignition::transport::Node node; */
/*   std::unordered_map<std::string, std::any> params = createDefaultParams(); */ 
/*   TurtleBot4_Nav env(params); */

/*   std::vector<double> ctrl_1 = {0.46, 0.0}; */
/*   int i = 0; */

/*   while (i < 300 && env.Terminal() == false) { */
/*       env.do_simulation(ctrl_1); */
/*       i+=1; */
/*   } */

/*   REQUIRE(env.Terminal() == true); */

/*   std::random_device rd; */
/*   std::mt19937 rng(rd()); */

/*   env.Reset(rng); */

/*   REQUIRE(env.Terminal() == false); */
/*   REQUIRE(abs(env.GetRobotX() - 0.0) < 0.01); */
/*   REQUIRE(abs(env.GetRobotY() - 0.0) < 0.01); */

/* } */

TEST_CASE("TurtleBot4 Simulation Step", "[sim_step]") {
    std::unordered_map<std::string, std::any> params = createDefaultParams();
    TurtleBot4_Nav env(params);

    std::vector<double> action = {0.1, -0.1}; // Example action input
    auto result = env.SimStep(action);
    
    REQUIRE(std::isfinite(result.r1));
    REQUIRE(env.step_ == 1); // Ensure step count is incremented
}

TEST_CASE("TurtleBot4 Get Observation", "[get_obs]") {
    std::unordered_map<std::string, std::any> params = createDefaultParams();
    TurtleBot4_Nav env(params);

    /* REQUIRE(reacher.m_ != nullptr); */
    /* REQUIRE(reacher.d_ != nullptr); */
    /* REQUIRE(reacher.d_->qpos != nullptr); */
    /* REQUIRE(reacher.d_->qvel != nullptr); */

    std::vector<double> obs(env.obs_size_, 0.0);
    env.get_obs(obs);

    std::vector<double> zero_obs(obs.size(), 0.0);
    REQUIRE(obs != zero_obs);

    /* REQUIRE(env.obs_size_ == params["cv_depth_height"] * params["cv_depth_width"] ); */


}

TEST_CASE("Turtlebot4 Action Bottleneck Test", "[action]") {
  std::unordered_map<std::string, std::any> params = createDefaultParams();
  TurtleBot4_Nav env(params);

  std::vector<double> action = {0.46, 0};

  std::random_device rd;
  std::mt19937 rng(rd());

  env.Reset(rng);

  auto result = env.SimStep(action);

  REQUIRE(abs(env.GetRobotX() - 0.045) < 0.04); 
  REQUIRE(abs(result.r1) < 5);


}
