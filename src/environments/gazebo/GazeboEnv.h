#ifndef GazeboEnv_h
#define GazeboEnv_h

#include "TaskEnv.h"

#include "cstdlib"
#include "iostream"
#include "thread"
#include "chrono"

using namespace std::chrono_literals;

#include "sys/types.h"
#include "sys/wait.h"
#include "misc.h"

//Ignition Gazebo
#include "ignition/msgs.hh"
#include "ignition/transport/Node.hh"
#include "ignition/msgs/pose.pb.h"
#include "ignition/msgs/boolean.pb.h"
#include "ignition/msgs/world_control.pb.h"
#include "ignition/msgs/entity_factory.pb.h"
#include "ignition/msgs/laserscan.pb.h"
#include "ignition/msgs/pose_v.pb.h"
#include "ignition/msgs/image.pb.h"

#include "opencv2/opencv.hpp"
#include "opencv2/rgbd.hpp"

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"

#include "sdf/Root.hh"
#include "sdf/World.hh"

//Image processing
#include "ignition/common.hh"


class GazeboEnv : public TaskEnv {
  public:
    sdf::Root root_;
    sdf::World *sdfWorld_ = nullptr;


    bool debug_msgs_ = false;
    int obs_size_;

    ignition::gazebo::ServerConfig config_;

    std::string worldFile_;
    std::string turtlebotPath_; 
    std::string object_path_; 

    /* Getting Robot Coordinates for Evaluator */





    float robot_yaw_{0.0};
    float robot_x_{0.0}; 
    float robot_y_{0.0};

    float obstacle_x_{0.0};
    float obstacle_y_{0.0};

    float target_x_{0.0};
    float target_y_{0.0};
    
    /* ----- Obstacle Sizing ----- */
    const double sx = 0.40;  // width
    const double sy = 0.40;  // thickness
    const double sz = 0.50;  // height
    const double cereal_z = 0.281525;

    float GetRobotX() { return robot_x_; }
    float GetRobotY() { return robot_y_; }
    float GetRobotYaw(){ return robot_yaw_;}

    float GetGoalX() { return target_x_; }
    float GetGoalY() { return target_y_; }

    float GetObstacleX() { return obstacle_x_;}
    float GetObstacleY() { return obstacle_y_;}

    float GetObstacleWidth() {return sx;}
    float GetObstacleDepth() {return sy;}



    static ignition::transport::NodeOptions MakeOpts() {
	    ignition::transport::NodeOptions opts;
	    opts.SetPartition(std::getenv("IGN_PARTITION"));
	    return opts;
    } 


    ignition::transport::Node node_{MakeOpts()};
    /* Getting optimal path length from plugin */



    double optimal_path_length_;
    double GetOptimalPathLength() { return optimal_path_length_;}


    float center_pixel_dist_ = 0; 

    float GetCenterPixel() { return center_pixel_dist_; } 




    /*   //define node, set ign partition then subcribe to topic */
    /*   //@TODO: set ign partition on action node? didn't i already do this? */


    






    /* publishing commands */
    ignition::transport::Node action_node_{MakeOpts()};
    std::string action_topic_ = "/model/turtlebot4/cmd_vel";
    ignition::transport::Node::Publisher action_pub_ = action_node_.Advertise<ignition::msgs::Twist>(action_topic_);

    std::unique_ptr<ignition::gazebo::Server> server_;

    /* no initial velocity */
    std::vector<float> init_pos_; 

    GazeboEnv() {}
    virtual ~GazeboEnv() {
      end_simulation();
    }

    virtual bool Terminal() = 0;
    virtual void Reset(std::mt19937& rng) = 0;
    virtual Results SimStep(std::vector<double>& action) = 0;

    void initialize_simulation() {



      ignition::msgs::EntityFactory req; 
      auto errors = root_.Load(worldFile_);

      if(!errors.empty()) {
	throw std::runtime_error("Failed to load SDF");
      }

      sdfWorld_ = root_.WorldByIndex(0);

      if(!sdfWorld_) {
	throw std::runtime_error("No world found");
      }
      if(debug_msgs_) std::cout << "World Name:  " << sdfWorld_->Name() << std::endl;


      config_.SetSdfFile(worldFile_);
      server_ = std::make_unique<ignition::gazebo::Server>(config_);
      std::cout << "Starting Simulation Server \n" << std::endl;



    
      bool result = false;

      while(!result) { 

	/* SPAWN TURTLEBOT */
	std::ifstream ifs(turtlebotPath_);

	std::string sdfXml((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>()); 

	ignition::msgs::EntityFactory request; 
	request.set_sdf(sdfXml);
	request.set_name("turtlebot4");
	/* request.set_allow_renaming(false); */
	
	ignition::msgs::Boolean response;

	server_->Run(true,1,false);

	if(!node_.Request("/world/maze/create", request, 500, response, result) || !result) {
	  if(debug_msgs_) std::cout << "Spawn Failed\n";
	}



      }

     
    }



    void end_simulation() {
      if(debug_msgs_) std::cout << "Requesting simulation shutdown \n" << std::endl;

      if(server_) {
	server_->Stop();
      }
    }
    
    


    void set_state(std::vector<float> qpos) {

      bool result = false;
      ignition::msgs::Pose pose_msg;

      pose_msg.set_name("turtlebot4");
      pose_msg.mutable_position()->set_x(qpos[0]);
      pose_msg.mutable_position()->set_y(qpos[1]);
      pose_msg.mutable_position()->set_z(0.01);
      pose_msg.mutable_orientation()->set_w(1.0);

      server_->Run(true,1,false);

      ignition::msgs::Boolean pose_rep;
      bool pose_success = node_.Request("/world/maze/set_pose", pose_msg, 1000, pose_rep, result);


      if(!pose_success || !result || !pose_rep.data()) {
	throw std::runtime_error("set_state failed");
      }

      server_->Run(true,10,false);

    }

    void do_simulation(std::vector<double>& ctrl){
      ignition::msgs::Twist ctrl_action; 


      ctrl_action.mutable_linear()->set_x(static_cast<float>(ctrl[0]));
      ctrl_action.mutable_linear()->set_y(0);
      ctrl_action.mutable_linear()->set_z(0);

      ctrl_action.mutable_angular()->set_x(0);
      ctrl_action.mutable_angular()->set_y(0);
      ctrl_action.mutable_angular()->set_z(static_cast<float>(ctrl[1]));

      action_pub_.Publish(ctrl_action);

      //running simulation for one step
      server_->Run(true,10,false);
    }

    int GetObsSize() { return obs_size_; }

};


#endif
