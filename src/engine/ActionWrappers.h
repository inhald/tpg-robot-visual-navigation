#ifndef ActionWrappers_h
#define ActionWrappers_h

#include "EvalData.h"

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <algorithm> // For std::transform
#include <chrono>
#include <random>

// TPG represents discrete actions as negative ints starting at -1
// Map them to positive ints starting at 0
inline int WrapDiscreteAction(EvalData &eval) {
    return (eval.program_out->action_ * -1) - 1;
}

inline double WrapContinuousAction(EvalData &eval) {
    return eval.program_out->private_memory_[MemoryEigen::kScalarType_]
        ->working_memory_[1](0, 0);
}

inline double WrapContinuousActionSigmoid(EvalData &eval) {
    double p = eval.program_out->private_memory_[MemoryEigen::kScalarType_]
                   ->working_memory_[1](0, 0);
    return 1 / (1 + exp(-p));
}

inline vector<double> WrapVectorAction(EvalData &eval) {
    auto mat = eval.program_out->private_memory_[MemoryEigen::kVectorType_]
                   ->working_memory_[1];
    vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
    return vec;
}

inline vector<double> WrapVectorActionSigmoid(EvalData &eval) {
    auto mat = eval.program_out->private_memory_[MemoryEigen::kVectorType_]
                   ->working_memory_[1];
    vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
    for (auto &v : vec) v = sigmoid(v);  // TODO(skelly): better/faster way?
    return vec;
}

inline vector<double> WrapVectorActionTanh(EvalData &eval) {
    auto mat = eval.program_out->private_memory_[MemoryEigen::kVectorType_]
                   ->working_memory_[1];
    vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
    for (auto &v : vec) v = std::tanh(v);  // TODO(skelly): better/faster way?
    return vec;
}

inline vector<double> WrapVectorActionMuJoco(EvalData &eval) {
    auto mat = eval.program_out->private_memory_[MemoryEigen::kVectorType_]
                   ->working_memory_[1];
    vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
    for (auto &v : vec) v = min(max(v, -1.0), 1.0);
    return vec;
}

inline int WrapDiscreteActionGazebo(EvalData &eval) {
    return (eval.program_out->action_ * -1) - 1;
}

inline vector<double> WrapVectorActionGazebo(EvalData &eval) {
    auto mat = eval.program_out->private_memory_[MemoryEigen::kVectorType_]
                   ->working_memory_[1];
    vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());

    /* std::cout << mat.rows() << std::endl; */
    /* std::cout << mat.cols() << std::endl; */

    //+- 0.46 = max linear velocity
    //+- 1.90 = max angular velocity
    /* for (auto &v : vec) v = min(max(v, -0.46), 0.46); */
    vector<double> res; 
    /* res.push_back(std::min(std::max(vec[0],-0.46), 0.46)); */
    /* res.push_back(std::min(std::max(vec[1],-1.90), 1.90)); */

    /* std::random_device rd; */
    /* std::mt19937 gen(rd()); */
    /* std::uniform_real_distribution<> distr1(-0.46,0.46); */
    /* std::uniform_real_distribution<> distr2(-1.90,1.90); */

    /* double lin_vel = distr1(gen); */
    /* double ang_vel = distr2(gen); */

    /* res.push_back(lin_vel); */
    /* res.push_back(ang_vel); */


    res.push_back(std::clamp(vec[0],-0.46, 0.46));
    res.push_back(std::clamp(vec[1],-1.90, 1.90));

    return res;
}

#endif
