/**
 * @file lite3_test_policy_runner.hpp
 * @brief 
 * @author mazunwang
 * @version 1.0
 * @date 2024-06-07
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef LITE3_TEST_POLICY_RUNNER_HPP_
#define LITE3_TEST_POLICY_RUNNER_HPP_

#include "policy_runner_base.hpp"
#include "torch/torch.h"
#include "torch/script.h"

class Lite3TestPolicyRunner : public PolicyRunnerBase
{
private:
    std::string policy_path_;

    torch::jit::Module backbone_;
    torch::Tensor action_tensor_, obs_total_tensor_;
    std::vector<c10::IValue> obs_vector_{};

    int obs_dim_ = 117;
    int obs_his_num_ = 40;
    int act_dim_ = 12;
    int obs_total_dim_;

    VecXf current_obs_, obs_history_, obs_total_;
    VecXf last_dof_pos0_, last_dof_pos1_, last_dof_pos2_;
    VecXf last_dof_vel0_, last_dof_vel1_;
    VecXf last_action1_, last_action0_, action_;

    VecXf dof_pos_default_;
    VecXf kp_, kd_;
    Vec3f max_cmd_vel_;

public:
    Lite3TestPolicyRunner(std::string policy_name):PolicyRunnerBase(policy_name){
        policy_path_ = GetAbsPath()+"/../policy/model_13000.pt";
        obs_total_dim_ = obs_dim_ + obs_his_num_*obs_dim_;
        dof_pos_default_.setZero(12);
        dof_pos_default_ << 0.0, -1, 1.8,
                            -0.0, -1, 1.8,
                            0.0, -1, 1.8,
                            -0.0, -1, 1.8;
        // kp_ = 20.*VecXf::Ones(12);
        // kd_ = 0.7*VecXf::Ones(12);

        // // for mujoco
        kp_ = 30.*VecXf::Ones(12);
        kd_ = 0.9 *VecXf::Ones(12);

        max_cmd_vel_ << 0.8, 0.3, 0.6;

        try { 
            backbone_ = torch::jit::load(policy_path_); 
            backbone_.eval();
        }
        catch (const c10::Error &e) { std::cerr << "error loading policy at " << policy_path_ << "\n" << e.what(); }

        for(int i=0;i<10;++i){
            torch::Tensor reaction;
            obs_vector_.clear();
            obs_vector_.emplace_back(torch::ones({1, obs_total_dim_}));  
            reaction = backbone_.forward(obs_vector_).toTensor();
            if(i==9) std::cout << policy_name_ << " network test success" << std::endl;
        }

        decimation_ = 12;
    }
    ~Lite3TestPolicyRunner(){
    }

    void DisplayPolicyInfo(){
        std::cout << "name : " << policy_name_ << std::endl; 
        std::cout << "path : " << policy_path_ << std::endl;
        std::cout << "dim  : " << obs_dim_ << " " << obs_his_num_ << " " << obs_total_dim_ << " " << act_dim_ << std::endl;
        std::cout << "dof  : " << dof_pos_default_.transpose() << std::endl;
        std::cout << "kp   : " << kp_.transpose() << std::endl;
        std::cout << "kd   : " << kd_.transpose() << std::endl;
        std::cout << "max_v: " << max_cmd_vel_.transpose() << std::endl;
    }
    

    void  OnEnter(){
        run_cnt_ = 0;
        current_obs_.setZero(obs_dim_);
        obs_history_.setZero(obs_dim_*obs_his_num_);
        obs_total_.setZero(obs_total_dim_); 
        std::cout << "enter " << policy_name_ << std::endl;
    }

    RobotAction GetRobotAction(const RobotBasicState& ro){
        Vec3f cmd_vel = ro.cmd_vel_normlized.cwiseProduct(max_cmd_vel_);

        if(run_cnt_ == 0){
            last_dof_pos2_ = last_dof_pos1_ = last_dof_pos0_ = ro.joint_pos;
            last_dof_vel1_ = last_dof_vel0_ = ro.joint_vel;
            last_action1_ = last_action0_ = ro.joint_pos - dof_pos_default_;
        }

        current_obs_.setZero(obs_dim_);
        current_obs_ << cmd_vel,
                        ro.base_rpy,
                        ro.base_omega,
                        ro.joint_pos,
                        0.1*ro.joint_vel,
                        last_dof_pos2_, 
                        last_dof_pos1_,
                        last_dof_pos0_,  // 72
                        0.1*last_dof_vel1_,
                        0.1*last_dof_vel0_,
                        last_action1_,
                        last_action0_;

        VecXf obs_history_record = obs_history_.segment(obs_dim_, (obs_his_num_-1)*obs_dim_).eval();
        obs_history_.segment(0, (obs_his_num_-1)*obs_dim_) = obs_history_record;
        obs_history_.segment((obs_his_num_-1)*obs_dim_, obs_dim_) = current_obs_;

        obs_total_.segment(0, obs_dim_) = current_obs_;
        obs_total_.segment(obs_dim_, obs_dim_*obs_his_num_) = obs_history_;

        last_dof_pos2_ = last_dof_pos1_;
        last_dof_pos1_ = last_dof_pos0_;
        last_dof_pos0_ = ro.joint_pos;

        last_dof_vel1_ = last_dof_vel0_;
        last_dof_vel0_ = ro.joint_vel;

        // ---------------------------------

        Eigen::MatrixXf temp = obs_total_.transpose();
        torch::Tensor a = torch::from_blob(temp.data(), {temp.rows(), temp.cols()}, torch::kFloat);
        obs_total_tensor_ = a.clone();

        obs_vector_.clear();
        obs_vector_.emplace_back(obs_total_tensor_);
        action_tensor_ = backbone_.forward(obs_vector_).toTensor();

        // Eigen::Matrix<float, 12, 1> act(action_tensor_.data_ptr<float>());
        Eigen::Map<Eigen::MatrixXf> act(action_tensor_.data_ptr<float>(), act_dim_, 1);
        
        // ------------------

        action_ = 0.25*act.col(0);

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

#endif

