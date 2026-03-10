/**
 * @file lite3_test_policy_runner_onnx.h
 * @brief Declaration only — Ort types are hidden via PIMPL so including this
 *        header does NOT pull in onnxruntime headers.  All implementation lives
 *        in lite3_test_policy_runner_onnx.cpp.
 * @author Bo (Percy) Peng
 * @version 1.0
 * @date 2025-08-10
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 */

#pragma once

#include "policy_runner_base.hpp"

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

using namespace types;

class Lite3TestPolicyRunnerONNX : public PolicyRunnerBase {
public:
    explicit Lite3TestPolicyRunnerONNX(std::string policy_name);
    ~Lite3TestPolicyRunnerONNX();

    void DisplayPolicyInfo() override;
    void OnEnter() override;
    RobotAction GetRobotAction(const RobotBasicState& ro) override;

private:
    // Hides all onnxruntime types so callers don't need to include
    // onnxruntime_cxx_api.h.
    struct OrtImpl;
    std::unique_ptr<OrtImpl> ort_;

    std::string model_path_;

    const int obs_dim_ = 45;
    const int act_dim_ = 12;

    VecXf current_obs_;
    VecXf joint_pos_rl;
    VecXf joint_vel_rl;
    VecXf last_action, tmp_action, action;

    VecXf dof_pos_default_robot, dof_pos_default_policy;
    VecXf kp_, kd_;
    Vec3f max_cmd_vel_;
    Vec3f gravity_direction;

    std::vector<int> robot2policy_idx, policy2robot_idx;

    float omega_scale_ = 0.25;
    float dof_vel_scale_ = 0.05;

    std::vector<std::string> robot_order;
    std::vector<std::string> policy_order;
    std::vector<float> action_scale_robot;

    RobotAction ra;

    std::vector<int> generate_permutation(
        const std::vector<std::string>& from,
        const std::vector<std::string>& to,
        int default_index = 0);
};
