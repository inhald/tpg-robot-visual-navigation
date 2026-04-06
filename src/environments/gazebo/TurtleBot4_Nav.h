#ifndef Turtlebot4_Nav
#define Turtlebot4_Nav

#define LIDAR_SCAN_SIZE 640

#define GOAL_X 2.00
#define GOAL_Y 0.00

#include "GazeboEnv.h"
#include "misc.h"
#include "mpi.h"

#include "cmath"
#include "vector"
#include "random"


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




class TurtleBot4_Nav : public GazeboEnv {
  private:
    ignition::transport::Node node_;
    ignition::transport::Node pose_node_;
    ignition::transport::Node lidar_node_;
    ignition::transport::Node rgb_node_;
    ignition::transport::Node depth_node_;
    ignition::transport::Node random_node_;
    ignition::transport::Node request_node_;

    std::string rgb_topic_ = "/world/maze/model/turtlebot4/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image";
    std::string depth_topic_ = "/world/maze/model/turtlebot4/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image";
    std::string lidar_scan_topic_ = "/world/maze/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan";
    std::string pose_topic_ = "/world/maze/pose/info";

    /* Sensor Measurements */
    std::string obs_type_;
    std::vector<float> cur_lidar_scan_;
    std::vector<uint8_t> cur_rgb_img_red_;
    std::vector<uint8_t> cur_rgb_img_green_;
    std::vector<uint8_t> cur_rgb_img_blue_;
    std::vector<float> cur_depth_feat_;

    int cv_depth_bins_ = 32;
    int num_rgb_feats_ = 3;

    std::mutex mtx1;
    std::mutex mtx2;
    std::mutex mtx3;

    std::mutex robot_mtx;

    //Video Stuff
    cv::VideoWriter video_;
    bool video_open_ = false;
    bool debug_video_ = false;
    double fps_ = 30.0;                         // set to your obs rate (e.g., 10.0)`
    int k_ = 4;                                 // your upsample factor
    std::string video_path_ = "/home/dhilan/tpg_codebase/tpg/experiments/gazebo_turtlebot4/logs/best_agent/depth_obs.mp4";
    std::string image_path_ = "/home/dhilan/tpg_codebase/tpg/experiments/gazebo_turtlebot4/logs/best_agent/depth_img";


    /* Collision Detection */
    bool collision_;
    int image_count = 0;
    int iter=0;

    /* Debug */
    std::atomic_bool wait_for_first_scan_ = false;
    std::atomic_bool wait_for_first_pose_ = false;


    /* Simulation Parameters */
    double reward_distance_weight_ = 1.0;
    double reward_control_weight_ = 0.1;
    double error_;


    /*CV Parameters*/
    int cv_depth_height_;
    int cv_depth_width_;
    int depth_obs_size_;
    const float keep_frac_ = 2.0f/3.0f;


    int cv_rgb_height_;
    int cv_rgb_width_;
    int rgb_obs_size_;

    /* Randomization */
    bool object_spawned_ = false;

    /* object */
    std::string sdfXml_o;

    /* --- Random Pose Start --- */
    std::atomic<uint64_t> next_req_id_{1};
    std::atomic<uint64_t> pending_req_id_{0};
    std::atomic<uint64_t> serviced_req_id_{0};
    std::atomic_bool request_serviced_{false};

    /* --- Random Pose Ends --- */

    /* RGB Detector */
    std::mutex rgb_mtx_;
    bool yellow_seen_{false};
    float yellow_cx_{0.0f};     // normalized horizontal offset in [-1, 1]
    float yellow_area_{0.0f};   // fraction of image in [0, 1]
    bool wait_for_first_rgb_{false};


    ignition::transport::Node::Publisher posePub_;
    ignition::transport::Node::Publisher ctrlPub_;
    ignition::transport::Node::Publisher requestPub_;

    float distance_to_object_ = 1000;
    float prev_distance_to_object_ = 1000;

    float theta_prev_ = 0, theta_cur_ = 0;
    float theta_ = 0;
    double error_cur_, error_prev_;
    double L_star_;


    //calculating reward

    int current_ep=0;
    int total_eps=10;

    double path_len_ = 0.0;
    double last_rx_ = std::numeric_limits<double>::quiet_NaN();
    double last_ry_ = std::numeric_limits<double>::quiet_NaN();
    double v_max_ = 0.46;
    double dt_ = 0.1;
    bool spl_awarded_ = false;


    //transformation matrices
    cv::Matx33f K_{277.f,   0.f, 160.f,
                   0.f, 277.f, 120.f,
                   0.f,   0.f,   1.f};

    cv::Matx33f R_bc_{cv::Matx33f::eye()};
    cv::Vec3f  t_bc_{0.f, 0.f, 0.f};

    const float RMAX = 12.0f;

    int replay;


    // Pick approximate cereal dimensions (meters): X x Y x Z
    /* const double sx = 0.40;  // width */
    /* const double sy = 0.40;  // thickness */
    /* const double sz = 0.50;  // height */
    /* const double cereal_z = 0.281525; */

    // Put COM on ground: z = half height
    /* const double x = 0/1* your x *1/; */
    /* const double y = 0/1* your y *1/; */
    /* const double z = 0; */

    //DEBUG
    std::atomic<uint64_t> dbg_req_sent_{0};
    std::atomic<uint64_t> dbg_req_rcvd_{0};
    


    //Mechanisms for stopping at target
    bool target_reached_ = false;

    const std::string sdfBox = R"(
    <sdf version="1.7">
      <model name="box">
	<static>true</static>
	<pose> 0 0 0 0 0 0 </pose>
	<link name="link">
	  <visual name="v">
	    <geometry><box><size>)" + std::to_string(sx) + " " + std::to_string(sy) + " " + std::to_string(sz) + R"(</size></box></geometry>
	    <material>
	      <ambient>0.8 0.5 0.2 1</ambient>
	      <diffuse>0.8 0.5 0.2 1</diffuse>
	      <specular>0.05 0.05 0.05 1</specular>
	      <pbr>
		<metal>
		  <metalness>0.0</metalness>
		  <roughness>0.8</roughness>
		  <alpha_mode>OPAQUE</alpha_mode>
		</metal>
	      </pbr>
	    </material>
	    <visibility_flags>0xFFFFFFFF</visibility_flags>
	  </visual>
	  <collision name="c">
	    <geometry><box><size>)" + std::to_string(sx) + " " + std::to_string(sy) + " " + std::to_string(sz) + R"(</size></box></geometry>
	  </collision>
	</link>
      </model>
    </sdf>
    )";



  public:
    TurtleBot4_Nav (std::unordered_map<std::string, std::any> &params) : node_(MakeOpts()), pose_node_(MakeOpts()), lidar_node_(MakeOpts()), rgb_node_(MakeOpts()), depth_node_(MakeOpts()), random_node_(MakeOpts()), request_node_(MakeOpts()) {
      //read in param file 
      obs_type_ = std::any_cast<std::string>(params["gz_obs_type"]);
      step_ = 0;

      total_eps = std::any_cast<int>(params["gz_n_eval_test"]);

      /* cv_depth_hi_ = std::any_cast<int>(params["cv_depth_hi"]); */
      /* cv_depth_hf_ = std::any_cast<int>(params["cv_depth_hf"]); */
      /* cv_depth_wi_ = std::any_cast<int>(params["cv_depth_wi"]); */
      /* cv_depth_wf_ = std::any_cast<int>(params["cv_depth_wf"]); */      

      cv_depth_height_ = std::any_cast<int>(params["cv_depth_height"]);
      cv_depth_width_ = std::any_cast<int>(params["cv_depth_width"]);
      
      cv_rgb_height_ = std::any_cast<int>(params["cv_rgb_height"]);
      cv_rgb_width_ = std::any_cast<int>(params["cv_rgb_width"]);

      
      eval_type_ = "Gazebo";
      n_eval_train_ = std::any_cast<int>(params["gz_n_eval_train"]);
      n_eval_validation_ = std::any_cast<int>(params["gz_n_eval_validation"]);
      n_eval_test_ = std::any_cast<int>(params["gz_n_eval_test"]);
      max_step_ = std::any_cast<int>(params["gz_max_timestep"]);
      reward_control_weight_ =
          std::any_cast<double>(params["gz_reward_control_weight"]);

      worldFile_ = ExpandEnvVars(std::any_cast<string>(params["gz_world_path"]));
      turtlebotPath_ = ExpandEnvVars(std::any_cast<string>(params["gz_model_path"]));
      object_path_ = ExpandEnvVars(std::any_cast<string>(params["gz_object_path"]));

      //map stuff
      replay = std::any_cast<int>(params["replay"]);

      //VIDEO STUFF



      //configuring object

      random_node_.Subscribe("/random_pose_res", &TurtleBot4_Nav::random_cb, this);

      //@TODO: Handle spawning using plugin

      std::ifstream ifs_o(object_path_);
      this->sdfXml_o = std::string(std::istreambuf_iterator<char>(ifs_o), std::istreambuf_iterator<char>());



      //pose pub

      posePub_ = request_node_.Advertise<ignition::msgs::Pose>("/world/maze/set_pose");

      ctrlPub_ = request_node_.Advertise<ignition::msgs::Pose>("/world/maze/control");

      requestPub_ = request_node_.Advertise<ignition::msgs::Boolean>("/random_pose_req");
      
     //configuring observations
      rgb_obs_size_ = cv_rgb_height_ * cv_rgb_width_;

      depth_obs_size_ = cv_depth_height_ * cv_depth_width_;

      if(obs_type_ == "lidar") {	
	obs_size_ = LIDAR_SCAN_SIZE;
	state_.resize(obs_size_);
      }
      if(obs_type_ == "depth") {
	/* obs_size_ = cv_depth_bins_ + 2; */
	obs_size_ = cv_depth_bins_ + num_rgb_feats_;
	/* obs_size_ = depth_obs_size_; */
	state_.resize(obs_size_);
      }
      if(obs_type_ == "rgb") {
	obs_size_ = 3*rgb_obs_size_;
	state_.resize(obs_size_);
      }

      //resizing
      cur_lidar_scan_.resize(LIDAR_SCAN_SIZE);
      cur_rgb_img_red_.resize(rgb_obs_size_);
      cur_rgb_img_green_.resize(rgb_obs_size_);
      cur_rgb_img_blue_.resize(rgb_obs_size_);

      //@TODO: FINISH RESIZING THEN PASS DEPTH TO OBS_VECT THEN DO SAME FOR RGB


      {
	std::lock_guard<std::mutex> lk(mtx1);

	cur_depth_feat_.assign(cv_depth_bins_,0.0f);

      }

      //initialize simulation and callbacks
      collision_ = false;
      initialize_simulation();
           


      /* if(!lidar_node_.Subscribe(lidar_scan_topic_, &TurtleBot4_Nav::lidar_cb, this)){ */
	/* if(debug_msgs_) std::cout << "Error subscribing to Lidar topic" << std::endl; */
      /* } */

      if(!pose_node_.Subscribe(pose_topic_, &TurtleBot4_Nav::pose_cb, this)){ 
	if(debug_msgs_) std::cout << "Error subscribing to pose topic" << std::endl;
      }

      if(obs_type_=="depth") {
	depth_node_.Subscribe(depth_topic_, &TurtleBot4_Nav::depth_cb, this);
      }


      //map callback
      if(replay) {
      node_.Subscribe("/path_length/optimal", &TurtleBot4_Nav::map_callback, this);
      }
      
      if(!rgb_node_.Subscribe(rgb_topic_, &TurtleBot4_Nav::rgb_cb, this)){
	if(debug_msgs_) std::cout << "Error subscribing to rgb topic" << std::endl;
      }





      //ensure lidar is receiving messages on the topic
      wait_for_first_scan_.store(WaitForFirstScan(), std::memory_order_release);


    }
    ~TurtleBot4_Nav() override {
    }

    /* - - - DEBUG  - - - */

    bool WaitForFirstScan(std::chrono::milliseconds timeout = std::chrono::seconds(4)) {

      auto start = std::chrono::steady_clock::now();

      const unsigned kIterPerCall = 10;          

      while (!wait_for_first_scan_.load(std::memory_order_acquire)) {
	server_->Run(true, kIterPerCall, /*paused=*/false);

	if (std::chrono::steady_clock::now() - start > timeout)
	  return false;                 
      }


      return true;                    
    }

    bool GetCollisionStatus() { return collision_; }



    void map_callback(const ignition::msgs::Double &map_msg) {

      optimal_path_length_ = map_msg.data();

    }


    bool isRunning() {return server_->Running(); }

    void SetSteps(unsigned int value) { step_ = value; }


    std::string getParam() {return obs_type_;}


    /* - - - END DEBUG  - - -*/


    void rgb_cb(const ignition::msgs::Image &msg) {

      const unsigned int width = msg.width();
      const unsigned int height = msg.height();

      if (msg.width() <= 0 || msg.height() <= 0) return;
      if (msg.step() <= 0) return;
      if (msg.data().empty()) return;


      const size_t bytes = msg.data().size();
      const size_t step = static_cast<size_t>(msg.step());

      if (bytes < step * static_cast<size_t>(height)) return;
      if (step < static_cast<size_t>(width) * 3) return;




      /* std::cout << "Received rgb msg\n"; */
      /* std::cout << "Width: " << width << " Height: " << height << std::endl; */

      const std::string &data = msg.data();

      if(data.empty()) return;

      const auto pixel_format = msg.pixel_format_type();


      bool is_rgb = false;
      bool is_bgr = false;


      if(pixel_format == ignition::msgs::PixelFormatType::RGB_INT8) is_rgb = true;
      else if (pixel_format == ignition::msgs::PixelFormatType::BGR_INT8) is_bgr = true;
      else return;

      cv::Mat bgr;


      if(is_rgb) {

	cv::Mat rgb(height, width, CV_8UC3, const_cast<char*>(msg.data().data()), step);
	cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      } 
      else if (is_bgr) {

	bgr = cv::Mat(height,width, CV_8UC3, const_cast<char*>(msg.data().data())).clone();

      }
      else {
	return;
      }


      cv::Mat hsv;
      cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);


      //yellow range
      cv::Scalar lower(20, 100, 80);
      cv::Scalar upper(35, 255, 255);


      cv::Mat mask;
      cv::inRange(hsv, lower, upper, mask);


      cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
      cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      bool seen = false;
      float cx = 0.0;
      float area_frac = 0.0f;

      double best_area = 0.0;
      cv::Moments best_moments{};


      for(const auto &c: contours){
	double a = cv::contourArea(c);
	if (a > best_area) {
	  best_area = a;
	  best_moments = cv::moments(c);
	}
      }

      if(best_area > 20.0 && best_moments.m00 > 1e-6) {

	double mx = best_moments.m10 / best_moments.m00;
	cx = static_cast<float>((mx - 0.5 * (width - 1)) / (0.5 * (width - 1)));
	area_frac = static_cast<float>(best_area / (width * height));
	seen = true;

      }

      {
	std::lock_guard<std::mutex> lk(rgb_mtx_);
	yellow_seen_ = seen;
	yellow_cx_ = seen ? cx : 0.0f;
	yellow_area_ = seen ? area_frac : 0.0f;
	wait_for_first_rgb_ = true;

      }



    }
    static uint64_t header_u64(const ignition::msgs::Header &h,
                           const std::string &key,
                           uint64_t fallback = 0)
    {
      for (const auto &kv : h.data()) {
	if (kv.key() == key && kv.value_size() > 0) {
	  try {
	    return std::stoull(kv.value(0));
	  } catch (...) {
	    return fallback;
	  }
	}
      }
      return fallback;
    }

    void random_cb(const ignition::msgs::Pose_V &random_msg)
    {
      const uint64_t resp_req_id =
	  header_u64(random_msg.header(), "req_id", 0);

      const uint64_t expected_req_id =
	  pending_req_id_.load(std::memory_order_acquire);

      if (resp_req_id == 0) {
	std::cerr << "[random_cb] Missing/invalid req_id in response\n";
	return;
      }

      if (expected_req_id == 0) {
	std::cerr << "[random_cb] No pending request, ignoring response req_id="
		  << resp_req_id << "\n";
	return;
      }

      if (resp_req_id != expected_req_id) {
	std::cerr << "[random_cb] Stale/mismatched response: got req_id="
		  << resp_req_id
		  << " expected=" << expected_req_id << "\n";
	return;
      }

      double tgt_x = 0.0, tgt_y = 0.0;
      double obs_x = 0.0, obs_y = 0.0;

      bool got_target = false;
      bool got_obstacle = false;

      for (const auto &p : random_msg.pose()) {
	if (p.name() == "target") {
	  tgt_x = p.position().x();
	  tgt_y = p.position().y();
	  got_target = true;
	}
	else if (p.name() == "obstacle") {
	  obs_x = p.position().x();
	  obs_y = p.position().y();
	  got_obstacle = true;
	}
      }

      if (!got_target || !got_obstacle) {
	std::cerr << "[random_cb] Missing pose(s): "
		  << "target=" << got_target
		  << " obstacle=" << got_obstacle
		  << " req_id=" << resp_req_id << "\n";
	return;
      }

      if (!std::isfinite(tgt_x) || !std::isfinite(tgt_y) ||
	  !std::isfinite(obs_x) || !std::isfinite(obs_y)) {
	std::cerr << "[random_cb] Non-finite coordinates in response req_id="
		  << resp_req_id << "\n";
	return;
      }

      {
	std::lock_guard<std::mutex> lock(mtx3);
	target_x_ = tgt_x;
	target_y_ = tgt_y;
	obstacle_x_ = obs_x;
	obstacle_y_ = obs_y;
      }

      serviced_req_id_.store(resp_req_id, std::memory_order_release);
      request_serviced_.store(true, std::memory_order_release);

      /* uint64_t got = dbg_req_rcvd_.fetch_add(1, std::memory_order_relaxed) + 1; */
      /* TRACE("[RANDRESP recv] count=" << got */
	    /* << " req_id=" << resp_req_id */
	    /* << " expected=" << expected_req_id */
	    /* << " target=(" << tgt_x << "," << tgt_y << ")" */
	    /* << " obstacle=(" << obs_x << "," << obs_y << ")" */
	    /* << " serviced=" << request_serviced_.load(std::memory_order_acquire)); */
    }

    /* void random_cb(const ignition::msgs::Pose_V &random_msg) { */
    /*   auto get_u64_header = [](const ignition::msgs::Header &h, */
			       /* const std::string &key, */
			       /* uint64_t fallback = 0) -> uint64_t { */
	/* for (const auto &kv : h.data()) { */
	  /* if (kv.key() == key && kv.value_size() > 0) { */
	    /* try { */
	      /* return std::stoull(kv.value(0)); */
	    /* } catch (...) { */
	      /* return fallback; */
	    /* } */
	  /* } */
	/* } */
	/* return fallback; */
    /*   }; */

    /*   const uint64_t resp_req_id = */
	  /* get_u64_header(random_msg.header(), "req_id", 0); */

    /*   const uint64_t expected_req_id = */
	  /* pending_req_id_.load(std::memory_order_acquire); */

    /*   if (resp_req_id == 0) { */
	/* std::cerr << "[random_cb] Missing/invalid req_id in response\n"; */
	/* return; */
    /*   } */

    /*   if (expected_req_id == 0) { */
	/* std::cerr << "[random_cb] No pending request, ignoring response req_id=" */
		  /* << resp_req_id << "\n"; */
	/* return; */
    /*   } */

    /*   if (resp_req_id != expected_req_id) { */
	/* std::cerr << "[random_cb] Stale/mismatched response: got req_id=" */
		  /* << resp_req_id */
		  /* << " expected=" << expected_req_id << "\n"; */
	/* return; */
    /*   } */

    /*   double tgt_x = 0.0, tgt_y = 0.0; */
    /*   double obs_x = 0.0, obs_y = 0.0; */

    /*   bool got_target = false; */
    /*   bool got_obstacle = false; */

    /*   for (const auto &p : random_msg.pose()) { */
	/* if (p.name() == "target") { */
	  /* tgt_x = p.position().x(); */
	  /* tgt_y = p.position().y(); */
	  /* got_target = true; */
	/* } else if (p.name() == "obstacle") { */
	  /* obs_x = p.position().x(); */
	  /* obs_y = p.position().y(); */
	  /* got_obstacle = true; */
	/* } */
    /*   } */

    /*   if (!got_target || !got_obstacle) { */
	/* std::cerr << "[random_cb] Missing pose(s): " */
		  /* << "target=" << got_target */
		  /* << " obstacle=" << got_obstacle */
		  /* << " req_id=" << resp_req_id << "\n"; */
	/* return; */
    /*   } */

    /*   if (!std::isfinite(tgt_x) || !std::isfinite(tgt_y) || */
	  /* !std::isfinite(obs_x) || !std::isfinite(obs_y)) { */
	/* std::cerr << "[random_cb] Non-finite coordinates in response req_id=" */
		  /* << resp_req_id << "\n"; */
	/* return; */
    /*   } */

    /*   { */
	/* std::lock_guard<std::mutex> lock(mtx3); */
	/* target_x_ = tgt_x; */
	/* target_y_ = tgt_y; */
	/* obstacle_x_ = obs_x; */
	/* obstacle_y_ = obs_y; */
    /*   } */

    /*   serviced_req_id_.store(resp_req_id, std::memory_order_release); */
    /*   request_serviced_.store(true, std::memory_order_release); */

    /*   uint64_t got = dbg_req_rcvd_.fetch_add(1, std::memory_order_relaxed) + 1; */
    /*   TRACE("[RANDRESP recv] count=" << got */
	    /* << " ep=" << current_ep */
	    /* << " req_id=" << resp_req_id */
	    /* << " expected=" << expected_req_id */
	    /* << " target=(" << tgt_x << "," << tgt_y << ")" */
	    /* << " obstacle=(" << obs_x << "," << obs_y << ")" */
	    /* << " serviced=" << request_serviced_.load(std::memory_order_acquire)); */
    /* } */




    void depthToBaseFrame(const cv::Mat& depth,
                      cv::Mat& points_base,
                      const cv::Matx33f& K,
                      const cv::Matx33f& R_bc,
                      const cv::Vec3f&  t_bc)
    {
	CV_Assert(depth.type() == CV_32FC1);
	const int H = depth.rows, W = depth.cols;

	// 1) Back-project: HxW CV_32FC3
	cv::Mat points_cam;
	cv::rgbd::depthTo3d(depth, K, points_cam);
	CV_Assert(points_cam.type() == CV_32FC3);
	CV_Assert(points_cam.rows == H && points_cam.cols == W);

	// 2) Flatten to Nx3 single-channel
	cv::Mat pts = points_cam.reshape(1, H*W);           // Nx3, CV_32F

	// 3) Rotate (row-vector convention → pts * R^T)
	cv::Mat pts_rot = pts * cv::Mat(R_bc.t());          // Nx3

	// 4) Translate: replicate t_bc into Nx3 and add
	cv::Mat t1x3(1, 3, CV_32F);
	t1x3.at<float>(0) = t_bc[0];
	t1x3.at<float>(1) = t_bc[1];
	t1x3.at<float>(2) = t_bc[2];
	cv::Mat t_row = cv::repeat(t1x3, H*W, 1);           // Nx3
	cv::Mat pts_base = pts_rot + t_row;                 // Nx3

	// 5) Restore to HxW with 3 channels
	points_base = pts_base.reshape(3, H);               // HxW CV_32FC3
    }


    void depth_cb (const ignition::msgs::Image &depth_msg) {
      

       /* if(!wait_for_first_scan_) return; */

	/* std::cout << "Received depth msg\n"; */

        int width = depth_msg.width();
        int height = depth_msg.height();

        if (width != 320 || height != 240) return;

	wait_for_first_scan_.store(true, std::memory_order_release);

        using PFT = ignition::msgs::PixelFormatType;
        const auto fmt  = depth_msg.pixel_format_type();
        const void* raw = depth_msg.data().data();
        const size_t step = static_cast<size_t>(depth_msg.step());


        cv::Mat depth_m; // owns memory
        if (fmt == PFT::R_FLOAT32) {
          cv::Mat rawf(height, width, CV_32FC1, const_cast<void*>(raw), step);
          depth_m = rawf.clone(); // own buffer
        }
        else if (fmt == PFT::L_INT16) {
          cv::Mat raw16(height, width, CV_16SC1, const_cast<void*>(raw), step);
          raw16.convertTo(depth_m, CV_32F, 1.0 / 1000.0); // mm -> m
        }
        else {
          std::cerr << "[depth_cb] Unsupported pixel format: " << fmt << "\n";
          return;
        }

        //sanitize

        cv::patchNaNs(depth_m, 0);
        const float max_range_m = 12.0f;
        cv::Mat valid = (depth_m > 0) & (depth_m < 1e6);
        depth_m.setTo(max_range_m, ~valid);
        cv::min(depth_m, max_range_m, depth_m);


	cv::Mat points_base;
	depthToBaseFrame(depth_m, points_base, K_, R_bc_, t_bc_);


	float fx = static_cast<float>(K_(0,0));
	int   W  = points_base.cols;
	float hfov = 2.0f * std::atan2(0.5f * W, fx);  // radians
	float a_min = -0.5f * hfov, a_max = 0.5f * hfov;


	int H = points_base.rows;
	int useH = (2*H)/3;     // ignore floor
	const int Kbins = 32;   // 16–48 works; 32 is a sweet spot

	std::vector<float> scan_m(Kbins, RMAX);  // meters; initialize to cap
	
	for (int v = 0; v < useH; ++v) {
	  const cv::Vec3f* row = points_base.ptr<cv::Vec3f>(v);
	  for (int u = 0; u < W; ++u) {
	    const cv::Vec3f& P = row[u];  // [X,Y,Z] in meters
	    float X = P[0], Y = P[1], Z = P[2];
	    if (!std::isfinite(Z) || Z <= 0.05f || Z > RMAX) continue;
	    if (Y < 0.05f || Y > 1.2f) continue;  // height gate (avoid floor/ceiling)

	    float az = std::atan2(X, Z);          // bearing: right positive
	    if (az < a_min || az > a_max) continue;
	    int b = (int)std::floor((az - a_min) * Kbins / (a_max - a_min));
	    if ((unsigned)b >= (unsigned)Kbins) continue;

	    float r = std::sqrt(X*X + Z*Z);       // range in meters
	    if (r < scan_m[b]) scan_m[b] = r;     // keep nearest
	  }
	}

	const size_t N = 32;

        {
	  std::lock_guard<std::mutex> lk(mtx1);

          if(cur_depth_feat_.size() != N) cur_depth_feat_.assign(N, 0.0f);

          std::memcpy(cur_depth_feat_.data(), scan_m.data(), N * sizeof(float));

        }




    }

    void pose_cb (const ignition::msgs::Pose_V &pose_msg) {

      wait_for_first_pose_.store(true, std::memory_order_release);

      for(int i = 0; i < pose_msg.pose_size(); ++i) {
	const auto &pose = pose_msg.pose(i);

	if(pose.name() == "turtlebot4") {

	  std::lock_guard<std::mutex> lk(robot_mtx);

	  robot_x_ = pose.position().x();
	  robot_y_ = pose.position().y();

	  ignition::math::Quaterniond q = ignition::msgs::Convert(pose.orientation());
	  robot_yaw_ = q.Yaw();

	  return;
	}
      }
    }


    void lidar_cb (const ignition::msgs::LaserScan &scan_msg) {

      cur_lidar_scan_.resize(scan_msg.ranges_size());
      collision_= false;

      for(int i=0; i < scan_msg.ranges_size(); ++i) {
	cur_lidar_scan_[i] = scan_msg.ranges(i);
      }

      for(int i = 0; i < scan_msg.ranges_size(); ++i) {
	if(abs(scan_msg.ranges(i)) < 0.25) {
	  collision_ = true;
	  /* std::cout << "[COLLISION DETECTED]!" << std::endl; */
	  return; 
	}
      }

    }




    //@TODO: When working with rgb, optimized the copying function 

    //note that callback rates for all three scans may be different

    void get_obs(std::vector<double> &obs) {

      obs.resize(obs_size_, 0.0);

      if(obs_type_ == "depth") {

	  std::vector<float> snap;
	  {
	    std::lock_guard<std::mutex> lk(mtx1);
	    snap = cur_depth_feat_;               // copy-out
	  }

	  // float -> double conversion; no temp vector needed
	  /* std::transform(snap.begin(), snap.end(), obs.begin(), */
			 /* [](float x){ return static_cast<double>(x); }); */

	  if(snap.size() >= 32) {

	    for(int i = 0; i < 32; ++i) {
	      obs[i] = static_cast<double>(snap[i]);
	    }

	  }
	  
	  bool seen;
	  float cx, area;
	  
	  {
	    std::lock_guard<std::mutex> lk(rgb_mtx_);
	    seen = yellow_seen_;
	    cx = yellow_cx_;
	    area = yellow_area_;
	  }
	  
	  obs[32] = seen ? 1.0: 0.0;
	  obs[33] = static_cast<double>(cx);
	  obs[34] = static_cast<double>(area);

      }

      if (obs.size() != static_cast<size_t>(obs_size_)) {
	/* TRACE("[OBS size mismatch] obs.size=" << obs.size() */
        /* << " obs_size_=" << obs_size_); */
	std::abort();
      }
      
      /* TRACE("[OBS fill] step=" << step_ */
	  /* << " size=" << obs.size() */
	  /* << " depth0=" << (obs.size() > 0 ? obs[0] : -999.0) */
	  /* << " seen=" << (obs.size() > 32 ? obs[32] : -999.0) */
	  /* << " cx=" << (obs.size() > 33 ? obs[33] : -999.0) */
	  /* << " area=" << (obs.size() > 34 ? obs[34] : -999.0)); */

    }


    /* bool wait_for_request(std::chrono::milliseconds timeout = std::chrono::seconds(4)) { */

    /*   auto start = std::chrono::steady_clock::now(); */

    /*   const unsigned kIterPerCall = 10; */

    /*   while (!request_serviced_.load(std::memory_order_acquire)) { */
	/* server_->Run(true, kIterPerCall, *paused=* */

	/* if (std::chrono::steady_clock::now() - start > timeout) { */
	  /* if(debug_msgs_) std::cout << "did not receive message\n"; */
	  /* return false; */                 
	/* } */
    /*   } */

    /*   if(debug_msgs_) std::cout << "Confirm received message\n"; */


    /*   return true; */

    /* } */

    bool wait_for_request(std::chrono::milliseconds timeout = std::chrono::seconds(4))
    {
      auto start = std::chrono::steady_clock::now();
      const unsigned kIterPerCall = 10;

      const uint64_t expected =
	  pending_req_id_.load(std::memory_order_acquire);

      while (true) {
	server_->Run(true, kIterPerCall, false);

	if (request_serviced_.load(std::memory_order_acquire) &&
	    serviced_req_id_.load(std::memory_order_acquire) == expected) {
	  return true;
	}

	if (std::chrono::steady_clock::now() - start > timeout) {
	  std::cerr << "[wait_for_request] timeout waiting for req_id="
		    << expected
		    << " serviced_req_id="
		    << serviced_req_id_.load(std::memory_order_acquire)
		    << " serviced="
		    << request_serviced_.load(std::memory_order_acquire)
		    << "\n";
	  return false;
	}
      }
    }




    bool Terminal() { 

      
      /* if ( ( step_ >= max_step_) || (distance_to_object_ <= 0.30)) { */

	/* if(debug_msgs_) std::cout << "Finished\n"; */

	/* return true; */


      /* } */
      /* else { */

	/* return false; */
      /* } */


      if(step_ >= max_step_) return true;
      if(target_reached_) return true;
      return false;

    }


    double compute_center_error_from_scaled(const cv::Mat& scaled /* HxW, float in [0,1] */) {

      const int H = scaled.rows, W = scaled.cols;
      const int topH = (2*H)/3; // ignore ground band if you didn’t already
      double num = 0.0, den = 0.0;

      for (int v = 0; v < topH; ++v) {
	const float* row = scaled.ptr<float>(v);
	for (int u = 0; u < W; ++u) {
	  double w = std::max(0.0f, 1.0f - row[u]); // near=1, far=0
	  num += w * u;
	  den += w;
	}
      }

      if (den < 1e-6) return 0.0;                     // nothing salient → neutral
      double ubar = num / den;
      double e = (ubar - 0.5*(W-1)) / (0.5*W);        // [-1,1], right positive
      return std::clamp(e, -1.0, 1.0);
    }
    
    Results SimStep(std::vector<double>& action) {

      double rx, ry, ox, oy;
      { 
	std::scoped_lock lock(robot_mtx, mtx3);
	rx = robot_x_; ry = robot_y_;
	ox = target_x_; oy = target_y_;
      }

      distance_to_object_ = std::hypot(ox - rx, oy - ry);
      double reward = -distance_to_object_;


      bool seen;
      float cx, area;
      {
	std::lock_guard<std::mutex> lk(rgb_mtx_);
	seen = yellow_seen_;
	cx = yellow_cx_;
	area = yellow_area_;
      }


      bool declared_stop = (std::abs(action[0]) < 1e-6 && std::abs(action[1]) < 1e-6);
      bool visual_goal_ok = seen && std::abs(cx) < 0.20f && area > 0.03f;

      /* std::cout << "visual goal ok: " << visual_goal_ok << std::endl; */

      if(declared_stop) {

	if(visual_goal_ok) {

	  reward += 0.1 * max_step_ + 10.0;
	  target_reached_ = true;

	} else {

	  reward -= 1.0;

	}

      }


      if(distance_to_object_ <= 0.30) {
	reward += 2.0;
      }

      do_simulation(action);

      get_obs(state_);
      step_++;

      
      return {reward, 0.0};

    }

    uint64_t send_random_request(uint64_t spawn_seed)
    {
      const uint64_t req_id =
	  next_req_id_.fetch_add(1, std::memory_order_relaxed);

      pending_req_id_.store(req_id, std::memory_order_release);
      serviced_req_id_.store(0, std::memory_order_release);
      request_serviced_.store(false, std::memory_order_release);

      ignition::msgs::Boolean req_msg;
      req_msg.set_data(true);

      auto *h = req_msg.mutable_header();

      auto *d1 = h->add_data();
      d1->set_key("spawn_seed");
      d1->add_value(std::to_string(spawn_seed));

      auto *d2 = h->add_data();
      d2->set_key("req_id");
      d2->add_value(std::to_string(req_id));

      requestPub_.Publish(req_msg);

      /* uint64_t sent = dbg_req_sent_.fetch_add(1, std::memory_order_relaxed) + 1; */
      /* TRACE("[RANDREQ send] count=" << sent */
	    /* << " req_id=" << req_id */
	    /* << " ep=" << current_ep */
	    /* << " seed=" << spawn_seed); */

      return req_id;
    }


    //@TODO: Random positions and Testing
    void Reset(std::mt19937& rng) {

      /* TRACE("[RESET begin] ep=" << current_ep */
      /* << " step=" << step_ */
      /* << " seen=" << yellow_seen_ */
      /* << " target=(" << target_x_ << "," << target_y_ << ")" */
      /* << " obstacle=(" << obstacle_x_ << "," << obstacle_y_ << ")"); */

      /* wait_for_first_scan_ = true; */

      ignition::msgs::Boolean world_response;
      ignition::msgs::Boolean pose_response;
      bool ok = false;

      //Step 1: Pause + Reset
      ignition::msgs::WorldControl pause;
      pause.set_pause(true);

      //Pause request
      request_node_.Request("/world/maze/control", pause, 500, world_response, ok);
      server_->Run(true, 1, false);


      //Step 3: Teleport Robot
      ignition::msgs::Pose pose_msg;
      pose_msg.set_name("turtlebot4");
      pose_msg.mutable_position()->set_x(0.0);
      pose_msg.mutable_position()->set_y(0.0);
      pose_msg.mutable_position()->set_z(0.01);
      pose_msg.mutable_orientation()->set_w(1.0);


      //Teleport request
      request_node_.Request("/world/maze/set_pose", pose_msg, 500, pose_response, ok);
      server_->Run(true,1,false);


      if(debug_msgs_) std::cout << "TURTLEBOT POSE RESET DONE\n"; 

      //Step 4: Stop the Robot
      /* auto pub = request_node_.Advertise<ignition::msgs::Twist>("/model/turtlebot4/cmd_vel"); */
      /* ignition::msgs::Twist zero; */
      /* zero.mutable_linear()->set_x(0.0); */
      /* zero.mutable_angular()->set_z(0.0); */
      /* pub.Publish(zero); */

      /* server_->Run(true,10,false); */


      //Step 5: Unpause 
      ignition::msgs::WorldControl unpause_msg;
      unpause_msg.set_pause(false);

      /* //Unpause request */
      request_node_.Request("/world/maze/control", unpause_msg, 500, world_response, ok);
      server_->Run(true, 1, false);

      /* /1* Step 6: Spawning Box Randomly *1/ */
      /* request_serviced_.store(false, std::memory_order_release); */

      /* ignition::msgs::Boolean random_req; */ 
      /* random_req.set_data(true); */

      /* auto* hdr = random_req.mutable_header(); */
      /* auto* d   = hdr->add_data(); */ 
      /* d->set_key("round"); */
      /* d->add_value(std::to_string(0)); */

      /* d = hdr->add_data(); */
      /* d->set_key("ep"); */
      /* d->add_value(std::to_string(current_ep)); */

      /* // NEW: spawn seed (64-bit) */
      uint64_t spawn_seed =
	  (static_cast<uint64_t>(rng()) << 32) | rng();

      const uint64_t req_id = send_random_request(spawn_seed);

      server_->Run(true, 1, false);
      /* d = hdr->add_data(); */
      /* d->set_key("spawn_seed"); */
      /* d->add_value(std::to_string(spawn_seed)); */


      /* uint64_t req_id = next_req_id_.fetch_add(1, std::memory_order_relaxed); */
      /* serviced_req_id_.store(0, std::memory_order_release); */
      /* pending_req_id_.store(req_id, std::memory_order_release); */
      /* request_serviced_.store(false, std::memory_order_release); */

      /* auto* d4 = hdr->add_data(); */
      /* d4->set_key("req_id"); */
      /* d4->add_value(std::to_string(req_id)); */

      /* uint64_t sent = dbg_req_sent_.fetch_add(1, std::memory_order_relaxed) + 1; */
      /* TRACE("[RANDREQ send] count=" << sent */
      /* << " req_id=" << req_id */
      /* << " ep=" << current_ep */
      /* << " seed=" << spawn_seed); */

      ok = wait_for_request(std::chrono::seconds(4));
      if (!ok) {
	std::cerr << "[RESET] randomization request failed for req_id="
            << req_id << "\n";
      }



      /* requestPub_.Publish(random_req); */


      /* TRACE("[RESET after server step] ep=" << current_ep */
      /* << " req_sent=" << dbg_req_sent_.load()); */



      bool model_result = false;


      /* Need to wait for request to be serviced */ 
      /* if(debug_msgs_) std::cout << "Sending pose request\n"; */
      /* if(!wait_for_request()) { */

	/* if(debug_msgs_) std::cout << "Random Pose timeout\n"; */
	/* return; */

      /* } */

      double obs_x, obs_y, tgt_x, tgt_y;


      {
	
	std::lock_guard<std::mutex> lock(mtx3);
	tgt_x = target_x_;
	tgt_y = target_y_;
	obs_x = obstacle_x_;
	obs_y = obstacle_y_;

      }




      if(!object_spawned_) {


	// ---  Spawning Cereal Box Start ---

	ignition::msgs::EntityFactory model_req;

	model_req.set_sdf(sdfXml_o);
	model_req.set_name("cereal"); //HARDCODED FOR NOW
	/* model_req.set_allow_renaming(false); */ 

	ignition::msgs::Pose object_pose;
	object_pose.mutable_position()->set_x(tgt_x);
	object_pose.mutable_position()->set_y(tgt_y);
	object_pose.mutable_position()->set_z(0.00);
	/* object_pose.mutable_position()->set_x(2); */
	/* object_pose.mutable_position()->set_y(0); */

	double yaw = M_PI * 0.5; // 90 deg
	ignition::math::Quaterniond q(0, 0, yaw);

	ignition::msgs::Set(object_pose.mutable_orientation(), q);


	*model_req.mutable_pose() = object_pose;


	if(!request_node_.Request("/world/maze/create", model_req, 500, world_response, model_result) || !model_result) {
	  if(debug_msgs_) std::cout << "Spawn Failed\n";
	}

	server_->Run(true,1,false);

	if(debug_msgs_) std::cout << "SPAWNED CEREAL BOX\n";


	// --- Spawning Cereal Box Ends ---
	

	// --- Spawning Obstacle Starts ---

	ignition::msgs::EntityFactory obstacle_req;
	obstacle_req.set_sdf(sdfBox);
	obstacle_req.set_name("box"); 

	ignition::msgs::Pose obstacle_pose;
	obstacle_pose.mutable_position()->set_x(obs_x);
	obstacle_pose.mutable_position()->set_y(obs_y);
	obstacle_pose.mutable_position()->set_z(sz * 0.5 + 0.01);
	yaw = M_PI * 0.5; // 90 deg
	ignition::math::Quaterniond q2(0, 0, yaw);

	ignition::msgs::Set(obstacle_pose.mutable_orientation(), q2);

	*obstacle_req.mutable_pose() = obstacle_pose;

	if(!request_node_.Request("/world/maze/create", obstacle_req, 500, world_response, model_result) || !model_result) {
	  std::cout << "Spawn Failed\n";
	}

	server_->Run(true,1,false);

	// --- Spawning Obstacle Ends ---

	object_spawned_ = true;


      }
      else {

	// --- Move Cereal Box Start ---

	ignition::msgs::Pose move_object;
	move_object.mutable_position()->set_x(tgt_x);
	move_object.mutable_position()->set_y(tgt_y);
	move_object.mutable_position()->set_z(0.00);
	move_object.set_name("cereal"); 

	double yaw = M_PI * 0.5; // 90 deg
	ignition::math::Quaterniond q(0, 0, yaw);
	ignition::msgs::Set(move_object.mutable_orientation(), q);


	if(!request_node_.Request("/world/maze/set_pose", move_object, 500, world_response, model_result) || !model_result) {
	  if(debug_msgs_) std::cout << "Move Failed\n";
	}


	if(debug_msgs_) std::cout << "Finished Moving Cereal Box to: " << tgt_x << "  " << tgt_y << std::endl;
	
	server_->Run(true,1,false);
	// --- Move Cereal Box End ---

	// --- Move Obstacle Box Start ---
	

	ignition::msgs::Pose move_box;
	move_box.mutable_position()->set_x(obs_x);
	move_box.mutable_position()->set_y(obs_y);
	move_box.mutable_position()->set_z(sz * 0.5 + 0.01);
	move_box.set_name("box"); 

	yaw = M_PI * 0.5; // 90 deg
	ignition::math::Quaterniond q_box(0, 0, yaw);
	ignition::msgs::Set(move_box.mutable_orientation(), q_box);

	/* std::cout << "Random Box Coords: " << box_x << "  " << box_y << std::endl; */

	if(!request_node_.Request("/world/maze/set_pose", move_box, 500, world_response, model_result) || !model_result) {
	  if(debug_msgs_) std::cout << "Move Failed\n";
	}

	server_->Run(true,1,false);

	// --- Move Obstacle Box End ---

      }

      /* //updating simulation parameters */
      step_ = 0;
      distance_to_object_ = 1000;
      collision_ = false;
      target_reached_ = false;
      
      {
	std::lock_guard<std::mutex> lk(rgb_mtx_);
	yellow_seen_ = false;
	yellow_cx_ = 0.0f;
	yellow_area_ = 0.0f;
	wait_for_first_rgb_ = false;
      }
      
      {
	std::lock_guard<std::mutex> lk(mtx1);
	cur_depth_feat_.assign(32, RMAX);
      }

      server_->Run(true,1,false);


      /* path_len_ = 0.0; */
      /* spl_awarded_ = false; */

      get_obs(state_);

      /* TRACE("[RESET end] ep=" << current_ep */
      /* << " obs_size=" << state_.size() */
      /* << " obs0=" << (state_.empty() ? -999.0 : state_[0]) */
      /* << " obs32=" << (state_.size() > 32 ? state_[32] : -999.0) */
      /* << " obs33=" << (state_.size() > 33 ? state_[33] : -999.0) */
      /* << " obs34=" << (state_.size() > 34 ? state_[34] : -999.0)); */

      current_ep +=1;
      current_ep = current_ep % total_eps;

    }

};

#endif


