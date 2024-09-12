/**
 * @file state_base.h
 * @brief state base class for robot state control
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef STATE_BASE_H_
#define STATE_BASE_H_

#include "common_types.h"
#include "custom_types.h"
#include "robot_interface.h"
#include "user_command_interface.h"
#include "data_streaming.hpp"
#include "basic_function.hpp"
#include "parameters/control_parameters.h"

using namespace types;
using namespace interface;
using namespace functions;

struct ControllerData{
    std::shared_ptr<RobotInterface> ri_ptr;
    std::shared_ptr<UserCommandInterface> uc_ptr;
    std::shared_ptr<ControlParameters> cp_ptr;
    std::shared_ptr<DataStreaming> ds_ptr;
};


class StateBase{
private:

public:
    StateBase(const RobotType& robot_type, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):
        robot_type_(robot_type),
        state_name_(state_name),
        data_ptr_(data_ptr){
            ri_ptr_ = data_ptr_->ri_ptr;
            uc_ptr_ = data_ptr_->uc_ptr;
            cp_ptr_ = data_ptr_->cp_ptr;
            ds_ptr_ = data_ptr_->ds_ptr;
            std::memset(&msfb_, 0, sizeof(msfb_));
        }

    ~StateBase(){}

    /**
     * @brief execute function when enter state
     */
    virtual void OnEnter() = 0;

    /**
     * @brief execute function when exit state
     */
    virtual void OnExit() = 0;

    /**
     * @brief run this function when in this state
     */
    virtual void Run() = 0;

    /**
     * @brief lose control judgement in every time stamp
     * @return true     robot is losing control and going to switch state
     * @return false    robot is in normal state
     */
    virtual bool LoseControlJudge() = 0;

    /**
     * @brief get the next state name if you want to switch state
     * @return StateName    next state name
     */
    virtual StateName GetNextStateName() = 0;

    /**
     * @brief state name for print 
     */
    std::string state_name_;

    /**
     * @brief robot_type for print
     */
    RobotType robot_type_;

    std::shared_ptr<ControllerData> data_ptr_;      //control data pointer 
    std::shared_ptr<RobotInterface> ri_ptr_;        //robot interface pointer to control your robot
    std::shared_ptr<UserCommandInterface> uc_ptr_;  //control your robot by design user command
    std::shared_ptr<ControlParameters> cp_ptr_;     //control parameters
    std::shared_ptr<DataStreaming> ds_ptr_;         //for data record

    static MotionStateFeedback msfb_;               //robot current motion state
};



#endif