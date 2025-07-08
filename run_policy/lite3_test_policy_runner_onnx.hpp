/**
 * @file lite3_test_policy_runner_onnx.hpp
 * @brief 
 * @author mayuxuan
 * @version 1.0
 * @date 2025-05-10
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */



#ifndef LITE3_TEST_POLICY_RUNNER_ONNX_HPP_
#define LITE3_TEST_POLICY_RUNNER_ONNX_HPP_


#include "policy_runner_base.hpp"
#include <onnxruntime_cxx_api.h>

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <iostream>

using namespace types;

class Lite3TestPolicyRunnerONNX : public PolicyRunnerBase {
private:
    std::string model_path_;

    Ort::Env env_;
    Ort::SessionOptions session_options_;
    Ort::Session session_;
    Ort::MemoryInfo memory_info_;

    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;

    int obs_dim_ = 117;
    int obs_his_num_ = 40;
    int act_dim_ = 12;
    int obs_total_dim_;

    VecXf current_obs_, obs_history_, obs_total_;
    VecXf last_dof_pos0_, last_dof_pos1_, last_dof_pos2_;
    VecXf last_dof_vel0_, last_dof_vel1_;
    VecXf last_action0_, last_action1_, action_;

    VecXf dof_pos_default_;
    VecXf kp_, kd_;
    Vec3f max_cmd_vel_;

public:
    Lite3TestPolicyRunnerONNX(std::string policy_name) : PolicyRunnerBase(policy_name),
        env_(ORT_LOGGING_LEVEL_WARNING, "ONNXPolicy"),
        session_options_(),
        memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
        session_(nullptr)
        {
        
        
        // .onnx model需要单独生成
        model_path_ = GetAbsPath() + "/../policy/RMA/model_13000.onnx";
        obs_total_dim_ = obs_dim_ + obs_his_num_ * obs_dim_;
        
        
        // 调试信息
        std::cout << "[ONNX INIT] Loading model: " << model_path_ << std::endl;

        session_options_.SetIntraOpNumThreads(1);
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        session_ = Ort::Session(env_, model_path_.c_str(), session_options_);
        std::cout << "[ONNX INIT] Model loaded successfully.\n"; 
        

        input_names_ = {"obs"};
        output_names_ = {"action"};


        dof_pos_default_.setZero(12);
        dof_pos_default_ << 0.0, -1, 1.8,
                            -0.0, -1, 1.8,
                            0.0, -1, 1.8,
                            -0.0, -1, 1.8;
        kp_ = 20.*VecXf::Ones(12);
        kd_ = 0.7*VecXf::Ones(12);
        max_cmd_vel_ << 0.8, 0.8, 0.8;

        // Test the model
        for (int i = 0; i < 2; ++i) {
            VecXf dummy_input = VecXf::Ones(obs_total_dim_);
            std::array<int64_t, 2> input_shape{1, obs_total_dim_};

            Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                memory_info_, dummy_input.data(), obs_total_dim_, input_shape.data(), input_shape.size());

            auto reaction = session_.Run(Ort::RunOptions{nullptr},
                                               input_names_.data(), &input_tensor, 1,
                                               output_names_.data(), 1);
            std::cout << policy_name_ << " ONNX policy network test success" << std::endl;
        }

        decimation_ = 12;
    }

    ~Lite3TestPolicyRunnerONNX() {}

    void DisplayPolicyInfo() override {
        std::cout << "ONNX policy: " << policy_name_ << "\n";
        std::cout << "path: " << model_path_ << "\n";
        std::cout << "obs_dim: " << obs_dim_ << ", history: " << obs_his_num_
                  << ", total: " << obs_total_dim_ << ", action_dim: " << act_dim_ << "\n";
    }

    void OnEnter() override {
        run_cnt_ = 0;
        current_obs_.setZero(obs_dim_);
        obs_history_.setZero(obs_dim_ * obs_his_num_);
        obs_total_.setZero(obs_total_dim_);
        std::cout << "[ONNX ENTER] PolicyRunner entered: " << policy_name_ << std::endl;
    }

    RobotAction GetRobotAction(const RobotBasicState& ro) override {
        Vec3f cmd_vel = ro.cmd_vel_normlized.cwiseProduct(max_cmd_vel_);

        if (run_cnt_ == 0) {
            last_dof_pos2_ = last_dof_pos1_ = last_dof_pos0_ = ro.joint_pos;
            last_dof_vel1_ = last_dof_vel0_ = ro.joint_vel;
            last_action1_ = last_action0_ = ro.joint_pos - dof_pos_default_;
        }

        current_obs_.setZero(obs_dim_);
        current_obs_ << cmd_vel,
                        ro.base_rpy,
                        ro.base_omega,
                        ro.joint_pos,
                        0.1 * ro.joint_vel,
                        last_dof_pos2_,
                        last_dof_pos1_,
                        last_dof_pos0_,
                        0.1 * last_dof_vel1_,
                        0.1 * last_dof_vel0_,
                        last_action1_,
                        last_action0_;

        if (run_cnt_ == 0) {
            for(int i = 0; i < obs_his_num_; ++i) {
                obs_history_.segment(i * obs_dim_, obs_dim_) = current_obs_;
            }
        }

        VecXf obs_history_record = obs_history_.segment(obs_dim_, (obs_his_num_ - 1) * obs_dim_);
        obs_history_.segment(0, (obs_his_num_ - 1) * obs_dim_) = obs_history_record;
        obs_history_.segment((obs_his_num_ - 1) * obs_dim_, obs_dim_) = current_obs_;

        obs_total_.segment(0, obs_dim_) = current_obs_;
        obs_total_.segment(obs_dim_, obs_dim_ * obs_his_num_) = obs_history_;

        last_dof_pos2_ = last_dof_pos1_;
        last_dof_pos1_ = last_dof_pos0_;
        last_dof_pos0_ = ro.joint_pos;

        last_dof_vel1_ = last_dof_vel0_;
        last_dof_vel0_ = ro.joint_vel;

        // ---------------------------------
        float* input_data = obs_total_.data();
        std::array<int64_t, 2> input_shape{1, obs_total_dim_};

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info_, input_data, obs_total_dim_, input_shape.data(), input_shape.size());

        auto output_tensors = session_.Run(Ort::RunOptions{nullptr},
                                           input_names_.data(), &input_tensor, 1,
                                           output_names_.data(), 1);

        float* action_data = output_tensors[0].GetTensorMutableData<float>();
        Eigen::Map<Eigen::MatrixXf> act(action_data, act_dim_, 1);

        // --------------------------------
        action_ = 0.25 * act.col(0);

        // std::cout << "[ONNX DEBUG] action (first 3 dims): "
        //           << action_(0) << ", " << action_(1) << ", " << action_(2) << std::endl;

        last_action1_ = last_action0_;
        last_action0_ = action_;

        RobotAction ra;
        ra.goal_joint_pos = action_ + dof_pos_default_;
        ra.goal_joint_vel = VecXf::Zero(act_dim_);
        ra.tau_ff = VecXf::Zero(act_dim_);
        ra.kp = kp_;
        ra.kd = kd_;
        ++run_cnt_;

        return ra;
    }
};

#endif  // LITE3_TEST_POLICY_RUNNER_ONNX_HPP_
