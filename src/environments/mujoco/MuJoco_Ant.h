#ifndef MuJoco_Ant_h
#define MuJoco_Ant_h

#include <MuJocoEnv.h>
#include <misc.h>

class MuJoco_Ant : public MuJocoEnv {
  public:
   // Parameters
   double control_cost_weight_ = 0.5;
   bool use_contact_forces_ = false;
   double contact_cost_weight_ = 5e-4;
   double healthy_reward_ = 1.0;
   bool terminate_when_unhealthy_ = true;
   bool terminate_when_off_course_ = false;
   double forward_threshold_ = 20.0;
   std::vector<double> x_vel_history_;
   size_t x_vel_history_size_ = 20;
   std::vector<double> healthy_z_range_;
   std::vector<double> contact_force_range_;
   double reset_noise_scale_ = 0.1;
   bool exclude_current_positions_from_observation_ = true;
  
   MuJoco_Ant(std::unordered_map<std::string, std::any>& params) {
      eval_type_ = "MuJoco";
      n_eval_train_ = std::any_cast<int>(params["mj_n_eval_train"]);
      n_eval_validation_ = std::any_cast<int>(params["mj_n_eval_validation"]);
      n_eval_test_ = std::any_cast<int>(params["mj_n_eval_test"]);
      max_step_ = 500;//std::any_cast<int>(params["mj_max_timestep"]);
      if (params.find("mj_reward_control_weight") != params.end()) {
      control_cost_weight_ =
          std::any_cast<double>(params["mj_reward_control_weight"]);
      }
      if (params.find("mj_healthy_reward") != params.end()) {
      healthy_reward_ =
          std::any_cast<double>(params["mj_healthy_reward"]);
      }
      healthy_reward_ = std::any_cast<double>(params["mj_healthy_reward"]);
      // terminate_when_off_course_ =
      //     std::any_cast<int>(params["mj_terminate_when_off_course"]);
      // exclude_current_positions_from_observation_ = 
      //     std::any_cast<int>(params["mj_ant_exclude_position_from_observation"]);    
      model_path_ = ExpandEnvVars(
          std::any_cast<string>(params["mj_model_path"]) + "ant.xml");
      healthy_z_range_ = {0.2, 1.0};
      contact_force_range_ = {-1.0, 1.0};
      x_vel_history_.resize(x_vel_history_size_,0.0);
      frame_skip_ = 5;
      initialize_simulation();

      obs_size_ = 27;
      if (!exclude_current_positions_from_observation_)
         obs_size_ += 2;
      if (use_contact_forces_)
         obs_size_ += 84;

      state_.resize(obs_size_);
   }

   ~MuJoco_Ant() {
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

   bool terminal() {
      return step_ >= max_step_ ||
             (terminate_when_unhealthy_ && !is_healthy()) ||
             (terminate_when_off_course_ &&
              static_cast<size_t>(step_) > x_vel_history_.size() * 2 &&
              std::accumulate(x_vel_history_.begin(), x_vel_history_.end(),
                              0.0) < forward_threshold_);
   }

   Results sim_step(std::vector<double>& action) {
      auto x_pos_before = d_->qpos[0];
      do_simulation(action, frame_skip_);
      auto x_pos_after = d_->qpos[0];
      auto x_vel =
          (x_pos_after - x_pos_before) / (m_->opt.timestep * frame_skip_);
      x_vel_history_[step_ % x_vel_history_.size()] = x_vel;
      auto forward_reward = x_vel;
      auto rewards = forward_reward + healthy_reward();
      auto ctrl_cost = control_cost(action);
      auto costs = ctrl_cost;
      if (use_contact_forces_) {
         costs += contact_cost();
      }
      auto reward = rewards - costs;
      get_obs(state_);
      step_++;
      return {reward, 0.0};  // TODO(skelly): maybe add gym 'info' to results
   }

   void get_obs(std::vector<double>& obs) {
      if (exclude_current_positions_from_observation_) {
         std::copy_n(d_->qpos + 2, m_->nq - 2, obs.begin());
         std::copy_n(d_->qvel, m_->nv, obs.begin() + (m_->nq - 2));
      } else {
         std::copy_n(d_->qpos, m_->nq, obs.begin());
         std::copy_n(d_->qvel, m_->nv, obs.begin() + m_->nq);
      }
   }

   void reset(mt19937& rng) {
      std::uniform_real_distribution<> dis_pos(-reset_noise_scale_,
                                               reset_noise_scale_);
      std::vector<double> qpos(m_->nq);
      for (size_t i = 0; i < qpos.size(); i++) {
         qpos[i] = init_qpos_[i] + dis_pos(rng);
      }
      std::normal_distribution<double> dis_vel(0.0, reset_noise_scale_);
      std::vector<double> qvel(m_->nv);
      for (size_t i = 0; i < qvel.size(); i++) {
         qvel[i] = init_qvel_[i] + dis_vel(rng);
      }
      mj_resetData(m_, d_);
      set_state(qpos, qvel);
      step_ = 0;
      get_obs(state_);
   }
};

#endif
