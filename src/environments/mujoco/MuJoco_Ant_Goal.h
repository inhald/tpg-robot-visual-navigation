#ifndef MuJoco_Ant_Goal_h
#define MuJoco_Ant_Goal_h

#include <MuJocoEnv.h>
#include <misc.h>

class MuJoco_Ant_Goal : public MuJocoEnv {
  private:
   int id_torso_;
   int id_target_;
  public:
   // Parameters
   double reward_distance_weight_ = 1.0;
   double control_cost_weight_ = 0.5;
   bool use_contact_forces_ = false;
   double contact_cost_weight_ = 5e-4;
   double healthy_reward_ = 1.0;
   bool terminate_when_unhealthy_ = true;
   std::vector<double> healthy_z_range_;
   std::vector<double> contact_force_range_;
   double reset_noise_scale_ = 0.1;
   double solved_dist_threshold_ = 1.3;
   bool terminate_when_solved_ = false;
   bool solved_ = false;
   bool exclude_current_positions_from_observation_ = true;

   MuJoco_Ant_Goal(std::unordered_map<std::string, std::any>& params) {
      eval_type_ = "MuJoco";
      n_eval_train_ = std::any_cast<int>(params["mj_n_eval_train"]);
      n_eval_validation_ = std::any_cast<int>(params["mj_n_eval_validation"]);
      n_eval_test_ = std::any_cast<int>(params["mj_n_eval_test"]);
      max_step_ = std::any_cast<int>(params["mj_max_timestep"]);
      if (params.find("mj_reward_control_weight") != params.end()) {
         control_cost_weight_ =
             std::any_cast<double>(params["mj_reward_control_weight"]);
      }
      if (params.find("mj_healthy_reward") != params.end()) {
         healthy_reward_ = std::any_cast<double>(params["mj_healthy_reward"]);
      }
      model_path_ = ExpandEnvVars(
          std::any_cast<string>(params["mj_model_path"]) + "ant_goal.xml");
      healthy_z_range_ = {0.2, 10.0};
      contact_force_range_ = {-1.0, 1.0};
      frame_skip_ = 5;
      initialize_simulation();

      id_torso_ = mj_name2id(m_, mjOBJ_XBODY, "torso");
      id_target_ = mj_name2id(m_, mjOBJ_XBODY, "target");

      obs_size_ = exclude_current_positions_from_observation_ ? 31 : 33;
      if (use_contact_forces_)
         obs_size_ += 84;

      state_.resize(obs_size_);
   }

   ~MuJoco_Ant_Goal() {
      // Free visualization storage
      mjv_freeScene(&scn_);
      mjr_freeContext(&con_);

      // Free MuJoCo model and data
      mj_deleteData(d_);
      mj_deleteModel(m_);

      // Terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
      glfwTerminate();
#endif
   }

   double healthy_reward() {
      return static_cast<double>(is_healthy() || terminate_when_unhealthy_) *
             healthy_reward_;
   }

   double control_cost(std::vector<double>& action) {
      double cost = 0;
      for (auto& a : action)
         cost += a * a;
      return control_cost_weight_ * cost;
   }

   std::vector<double> contact_forces() {
      std::vector<double> forces;
      std::copy_n(d_->cfrc_ext, m_->nbody * 6, back_inserter(forces));
      for (auto& f : forces) {
         f = std::max(contact_force_range_[0],
                      std::min(f, contact_force_range_[1]));
      }
      return forces;
   }

   double contact_cost() {
      auto forces = contact_forces();
      double cost = 0;
      for (auto& f : forces)
         cost += f * f;
      return contact_cost_weight_ * cost;
   }

   bool is_healthy() {
      for (int i = 0; i < m_->nq; i++)
         if (!std::isfinite(d_->qpos[i]))
            return false;
      for (int i = 0; i < m_->nv; i++)
         if (!std::isfinite(d_->qvel[i]))
            return false;
      return (d_->qpos[2] >= healthy_z_range_[0] &&
              d_->qpos[2] <= healthy_z_range_[1]);        
   }

   std::vector<double> get_dist() {
      return {d_->xpos[id_torso_ * 3 + 0] - d_->xpos[id_target_ * 3 + 0],
              d_->xpos[id_torso_ * 3 + 1] - d_->xpos[id_target_ * 3 + 1]};
   }

   bool terminal() {
      return step_ >= max_step_ || (solved_ && terminate_when_solved_) || (terminate_when_unhealthy_ && !is_healthy());
   }

   Results sim_step(std::vector<double>& action) {
      auto dist_diff = get_dist();
      auto dist = std::sqrt(std::pow(dist_diff[0], 2) + std::pow(dist_diff[1], 2));
      // Check solved. Never reset during an episode
      if (dist <= solved_dist_threshold_)
         solved_ = true;

      double reward_dist = -reward_distance_weight_ * dist;

      double reward_ctrl = -control_cost(action);

      reward = reward_dist + reward_ctrl;
      do_simulation(action, frame_skip_);
      get_obs(state_);
      step_++;
      return {reward, (solved_ ? 1.0 :0.0)};  // TODO(skelly): maybe add gym 'info' to results
   }

   void get_obs(std::vector<double>& obs) {
         int j = 0;   
         // Distance between torso and target (2 var)
         auto dist_diff = get_dist();  
         obs[j++] = dist_diff[0];
         obs[j++] = dist_diff[1];
         // Target position (2 var)
         obs[j++] = d_->xpos[id_target_ * 3 + 0];
         obs[j++] = d_->xpos[id_target_ * 3 + 1];
         // Regular ant obs, maybe including torso position (27/29 var)
         for (int i = (exclude_current_positions_from_observation_ ? 2 : 0); 
            i < 15; i++) {
            obs[j++] = d_->qpos[i];
         }
         for (int i = 0; i < 14; i++) {
            obs[j++] = d_->qvel[i];
         }
   }

   void reset(mt19937& rng) {
      // Body
      std::uniform_real_distribution<> dis_pos(-reset_noise_scale_,
                                               reset_noise_scale_);
      std::vector<double> qpos(m_->nq);
      for (size_t i = 0; i < qpos.size(); i++) {
         qpos[i] = init_qpos_[i] + dis_pos(rng);
      }

      std::vector<double> goal(2);

      // Goal
      std::uniform_real_distribution<> dis_goal(-5.0, 5.0);
      do {
         goal[0] = dis_goal(rng);
         goal[1] = dis_goal(rng);
      } while (goal[0] * goal[0] < 1 || goal[1] * goal[1] < 1);

      // // Goal  on radius
      // std::uniform_real_distribution<> dis_angle(0.0, 2.0 * M_PI);
      // double radius = 2.0;
      // double angle = dis_angle(rng);
      // goal[0] = radius * std::cos(angle);
      // goal[1] = radius * std::sin(angle);

      int joint_id = mj_name2id(m_, mjOBJ_JOINT, "target_joint");
      int qpos_address = m_->jnt_qposadr[joint_id];
      qpos[qpos_address + 0] = goal[0];
      qpos[qpos_address + 1] = goal[1];
      qpos[qpos_address + 2] = 0.25;

      std::normal_distribution<double> dis_vel(0.0, reset_noise_scale_);
      std::vector<double> qvel(m_->nv);
      for (size_t i = 0; i < qvel.size(); i++) {
         qvel[i] = init_qvel_[i] + dis_vel(rng);
      }
      mj_resetData(m_, d_);
      set_state(qpos, qvel);
      step_ = 0;
      solved_ = false;
      get_obs(state_);
   }
};

#endif
