/**
 * @file idle_state.hpp
 * @brief robot need to confirm sensor input while in idle state
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef IDLE_STATE_HPP_
#define IDLE_STATE_HPP_


#include "state_base.h"

class IdleState : public StateBase{
private:
    bool joint_normal_flag_ = false, imu_normal_flag_ = false;
    bool first_enter_flag_ = true;
    VecXf joint_pos_, joint_vel_, joint_tau_;
    Vec3f rpy_, acc_, omg_;
    double enter_state_time_ = -10000.;

    void GetProprioceptiveData(){
        joint_pos_ = ri_ptr_->GetJointPosition();
        joint_vel_ = ri_ptr_->GetJointVelocity();
        joint_tau_ = ri_ptr_->GetJointTorque();
        rpy_ = ri_ptr_->GetImuRpy();
        acc_ = ri_ptr_->GetImuAcc();
        omg_ = ri_ptr_->GetImuOmega();
    }

    bool JointDataNormalCheck(){
        VecXf joint_pos_lower(12), joint_pos_upper(12);
        Vec3f fl_lower = cp_ptr_->fl_joint_lower_;
        Vec3f fl_upper = cp_ptr_->fl_joint_upper_;
        Vec3f fr_lower(-fl_upper(0), fl_lower(1), fl_lower(2));
        Vec3f fr_upper(-fl_lower(0), fl_upper(1), fl_upper(2));
        joint_pos_lower << fl_lower, fr_lower, fl_lower, fr_lower;
        joint_pos_upper << fl_upper, fr_upper, fl_upper, fr_upper;
        for(int i=0;i<12;++i){
            if(std::isnan(joint_pos_lower(i)) || joint_pos_(i) > joint_pos_upper(i)+0.1 || joint_pos_(i) < joint_pos_lower(i)-0.1) {
                // std::cout << "joint pos " << i << " : " << joint_pos_(i) << " | " 
                //                                         << joint_pos_lower(i) << " " << joint_pos_upper(i) << std::endl;
                return false;
            }
            if(std::isnan(joint_vel_(i)) || (joint_vel_(i)) > cp_ptr_->joint_vel_limit_(i%3) + 0.1) {
                // std::cout << "joint vel " << i << " : " << joint_vel_(i) << " | " << cp_ptr_->joint_vel_limit_(i%3) << std::endl;
                return false;
            }
        }
        return true;
    }
    bool ImuDataNormalCheck(){
        for(int i=0;i<3;++i){
            if(std::isnan(rpy_(i)) || fabs(rpy_(i)) > M_PI){
                std::cout << "rpy_ " << i << " : " << rpy_(i) << std::endl;
                return false;
            } 
            if(std::isnan(omg_(i)) || fabs(omg_(i)) > M_PI){
                std::cout << "omg_ " << i << " : " << omg_(i) << std::endl;
                return false;
            }
        }
        if(acc_.norm() < 0.1*gravity || acc_.norm() > 3.0*gravity){
            std::cout << "acc " << " : " << acc_.transpose() << std::endl;
            return false;
        }
        return true;
    }

    void DisplayProprioceptiveInfo(){
        std::cout << "Joint Data: \n";
        std::cout << "pos: " << joint_pos_.transpose() << std::endl;
        std::cout << "vel: " << joint_vel_.transpose() << std::endl;
        std::cout << "tau: " << joint_tau_.transpose() << std::endl;
        std::cout << "Imu Data: \n";
        std::cout << "rpy: " << rpy_.transpose() << std::endl;
        std::cout << "acc: " << acc_.transpose() << std::endl;
        std::cout << "omg: " << omg_.transpose() << std::endl;
    }

    void DisplayAxisValue(){
        auto cmd = uc_ptr_->GetUserCommand();
        std::cout << "User Command Input: \n";
        std::cout << "axis value:  " << cmd.forward_vel_scale << " " 
                                     << cmd.side_vel_scale << " "
                                     << cmd.turnning_vel_scale << std::endl;
        std::cout << "target mode: " << cmd.target_mode << std::endl;
    }


public:
    IdleState(const RobotType& robot_type, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr){
        }
    ~IdleState(){}

    virtual void OnEnter() {
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::WaitingForStand);
        std::cout << "Waiting for stand up..." << std::endl;
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
        enter_state_time_ = ri_ptr_->GetInterfaceTimeStamp();
    };
    virtual void OnExit() {
        first_enter_flag_ = false;
    }
    virtual void Run() {
        GetProprioceptiveData();
        joint_normal_flag_ = JointDataNormalCheck();
        imu_normal_flag_ = ImuDataNormalCheck();
        if(first_enter_flag_ && ri_ptr_->GetInterfaceTimeStamp() - enter_state_time_ > 2.){//to confirm right state input
            DisplayProprioceptiveInfo();
            DisplayAxisValue();
        }
        MatXf cmd = MatXf::Zero(12, 5);
        ri_ptr_->SetJointCommand(cmd);
    }

    virtual bool LoseControlJudge() {
        return false;
    }

    virtual StateName GetNextStateName() {
        std::cout << "Current target_mode = " << uc_ptr_->GetUserCommand().target_mode << std::endl;

        if(!joint_normal_flag_ || !imu_normal_flag_) {
            std::cout << "joint status: " << joint_normal_flag_ << " | imu status: " << imu_normal_flag_ << std::endl;
            return StateName::kIdle;
        }
        if(first_enter_flag_ && ri_ptr_->GetInterfaceTimeStamp() - enter_state_time_ < 0.5){
            return StateName::kIdle;
        }
            
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::StandingUp)) return StateName::kStandUp;
        
        return StateName::kIdle;
    }
};




#endif