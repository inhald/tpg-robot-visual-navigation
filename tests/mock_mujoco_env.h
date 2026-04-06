#ifndef MOCK_MUJOCO_ENV_H
#define MOCK_MUJOCO_ENV_H

#include <random>
#include <string>
#include <vector>

#include "../src/environments/mujoco/MuJocoEnv.h"

class MockMuJocoEnv : public MuJocoEnv {
   public:
    MockMuJocoEnv(std::string model_path) {
        model_path_ = model_path;
    }

    void reset(std::mt19937& rng) override {}
    bool terminal() override { return false; }
    Results sim_step(std::vector<double>& action) override { return {}; }
};
#endif