/**
 * @file joint_damping_state.hpp
 * @brief joint passive control state
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef JOINT_DAMPING_STATE_HPP_
#define JOINT_DAMPING_STATE_HPP_

#include "state_base.h"


class JointDampingState : public StateBase{
private:
    float time_record_, run_time_;
    MatXf joint_cmd_;
public:
    JointDampingState(const RobotType& robot_type, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr){
            VecXf kd_ = cp_ptr_->swing_leg_kd_.replicate(4, 1);
            joint_cmd_ = MatXf::Zero(12, 5);
            joint_cmd_.col(2) = kd_;
        }
    ~JointDampingState(){}

    virtual void OnEnter() {
        time_record_ = ri_ptr_->GetInterfaceTimeStamp();
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
    };
    virtual void OnExit() {

    }
    virtual void Run() {
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        ri_ptr_->SetJointCommand(joint_cmd_);
    }
    virtual bool LoseControlJudge() {
        return false;
    }
    virtual StateName GetNextStateName() {
        if(run_time_ - time_record_ < 3.){
            return StateName::kJointDamping;
        }
        return StateName::kIdle;
    }
};





#endif