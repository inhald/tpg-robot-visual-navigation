#ifndef MuJoco_Half_Cheetah_h
#define MuJoco_Half_Cheetah_h

#include <MuJocoEnv.h>
#include <misc.h>

class MuJoco_Half_Cheetah : public MuJocoEnv {
  public:
   // Parameters
   double forward_reward_weight = 1.0;
   double control_cost_weight_ = 0.5;
   double reset_noise_scale_ = 0.1;
   bool exclude_current_positions_from_observation_ = true;

   MuJoco_Half_Cheetah(std::unordered_map<std::string, std::any>& params) {
      eval_type_ = "MuJoco";
      n_eval_train_ = std::any_cast<int>(params["mj_n_eval_train"]);
      n_eval_validation_ = std::any_cast<int>(params["mj_n_eval_validation"]);
      n_eval_test_ = std::any_cast<int>(params["mj_n_eval_test"]);
      max_step_ = 500;//std::any_cast<int>(params["mj_max_timestep"]);
      if (params.find("mj_reward_control_weight") != params.end()) {
      control_cost_weight_ =
          std::any_cast<double>(params["mj_reward_control_weight"]);
      }
      model_path_ =
          ExpandEnvVars(std::any_cast<string>(params["mj_model_path"]) + 
                        "half_cheetah.xml");   
      frame_skip_ = 5;              
      initialize_simulation();
      obs_size_ = 18;
      if (exclude_current_positions_from_observation_)
         obs_size_--;
      state_.resize(obs_size_);
   }

   ~MuJoco_Half_Cheetah() {
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

   double control_cost(std::vector<double>& action) {
      double cost = 0;
      for (auto& a : action)
         cost += a * a;
      return control_cost_weight_ * cost;
   }

   bool terminal() { return step_ >= max_step_; }

   Results sim_step(std::vector<double>& action) {
      auto x_pos_before = d_->qpos[0];
      do_simulation(action, frame_skip_);
      auto x_pos_after = d_->qpos[0];
      auto x_vel =
          (x_pos_after - x_pos_before) / (m_->opt.timestep * frame_skip_);

      auto ctrl_cost = control_cost(action);

      auto forward_reward = forward_reward_weight * x_vel;

      auto rewards = forward_reward;
      auto costs = ctrl_cost;

      get_obs(state_);

      auto reward = rewards - costs;
      step_++;
      return {reward, 0.0};  // TODO(skelly): maybe add gym 'info' to results
   }

   void get_obs(std::vector<double>& obs) {
      if (exclude_current_positions_from_observation_) {
         std::copy_n(d_->qpos + 1, m_->nq - 1, obs.begin());
         std::copy_n(d_->qvel, m_->nv, obs.begin() + (m_->nq - 1));
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
