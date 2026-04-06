#ifndef MuJoco_Walker2D_h
#define MuJoco_Walker2D_h

#include <MuJocoEnv.h>
#include <misc.h>
#include <iostream>

class MuJoco_Walker2D : public MuJocoEnv {
  public:
   // Parameters
   double forward_reward_weight_ = 1.0;
   double control_cost_weight_ = 1e-3;
   double healthy_reward_ = 1.0;
   bool terminate_when_unhealthy_ = true;
   std::vector<double> healthy_z_range_;
   std::vector<double> healthy_angle_range_;
   double reset_noise_scale_ = 5e-3;
   bool exclude_current_positions_from_observation_ = true;

   MuJoco_Walker2D(std::unordered_map<std::string, std::any>& params) {
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
      model_path_ = ExpandEnvVars(
          std::any_cast<string>(params["mj_model_path"]) + "walker2d.xml");
      healthy_z_range_ = {0.8, 2.0};
      healthy_angle_range_ = {-1.0, 1.0};
      frame_skip_ = 4;
      initialize_simulation();

      obs_size_ = 18;
      if (exclude_current_positions_from_observation_)
         obs_size_--;

      state_.resize(obs_size_);
   }

   ~MuJoco_Walker2D() {
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

   bool is_healthy() {
      double z = d_->qpos[1];
      double angle = d_->qpos[2];

      bool healthy_z = healthy_z_range_[0] < z && z < healthy_z_range_[1];
      bool healthy_angle =
          healthy_angle_range_[0] < angle && angle < healthy_angle_range_[1];
      return healthy_z && healthy_angle;
   }

   bool terminal() {
      return step_ >= max_step_ || (terminate_when_unhealthy_ && !is_healthy());
   }

   Results sim_step(std::vector<double>& action) {
      auto x_pos_before = d_->qpos[0];
      do_simulation(action, frame_skip_);
      auto x_pos_after = d_->qpos[0];
      auto x_vel =
          (x_pos_after - x_pos_before) / (m_->opt.timestep * frame_skip_);
      auto ctrl_cost = control_cost(action);

      auto forward_reward = forward_reward_weight_ * x_vel;
      auto rewards = forward_reward + healthy_reward();

      auto costs = ctrl_cost;
      auto reward = rewards - costs;

      get_obs(state_);
      step_++;
      return {reward, 0.0};
   }

   void get_obs(std::vector<double>& obs) {
      auto position_size =
          exclude_current_positions_from_observation_ ? m_->nq - 1 : m_->nq;

      if (exclude_current_positions_from_observation_) {
         std::copy_n(d_->qpos + 1, position_size, state_.begin());
      } else {
         std::copy_n(d_->qpos, position_size, state_.begin());
      }
      std::copy_n(d_->qvel, m_->nv, state_.begin() + position_size);

      for (int i = 0; i < m_->nv; ++i) {
         state_[position_size + i] =
             std::clamp(state_[position_size + i], -10.0, 10.0);
      }
   }

   void reset(mt19937& rng) {
      std::uniform_real_distribution<> dis_pos(-reset_noise_scale_,
                                               reset_noise_scale_);
      std::vector<double> qpos(m_->nq);
      for (size_t i = 0; i < qpos.size(); i++) {
         qpos[i] = init_qpos_[i] + dis_pos(rng);
      }
      std::uniform_real_distribution<> dis_vel(-reset_noise_scale_,
                                               reset_noise_scale_);
      std::vector<double> qvel(m_->nv);
      for (size_t i = 0; i < qvel.size(); i++) {
         qvel[i] = init_qvel_[i] + dis_vel(rng);
      }
      mj_resetData(m_, d_);
      set_state(qpos, qvel);
      step_ = 0;
      get_obs(state_);
   };
};

#endif
