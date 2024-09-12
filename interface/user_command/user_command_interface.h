/**
 * @file user_command_interface.h
 * @brief this file is used for robot's user command input
 * @author mazunwang
 * @version 1.0
 * @date 2024-04-11
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef USER_COMMAND_INTERFACE_H_
#define USER_COMMAND_INTERFACE_H_

#include "common_types.h"
#include "custom_types.h"

using namespace types;

namespace interface{

class UserCommandInterface{
private:
    /* data */
public:
    UserCommandInterface(){}
    ~UserCommandInterface(){}

    /**
     * @brief start the thread to process user command
     */
    virtual void Start() = 0;

    /**
     * @brief stop the thread 
     */
    virtual void Stop() = 0;

    /**
     * @brief return your user command
     * @return UserCommand 
     */
    virtual UserCommand GetUserCommand() = 0; 

    /**
     * @brief set the motion state feedback 
     * @param  msfb         motion state feedback
     */
    virtual void SetMotionStateFeedback(const MotionStateFeedback& msfb) = 0;

    MotionStateFeedback msfb_;
};
};

#endif