/**
 * @file control_parameters.h
 * @brief basic control parameters
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef CONTROL_PARAMETERS_HPP_
#define CONTROL_PARAMETERS_HPP_

#include "common_types.h"
#include "custom_types.h"

using namespace types;

class ControlParameters
{
private:
    void GenerateLite3Parameters();

public:
    ControlParameters(RobotType robot_type){
        if(robot_type==RobotType::Lite3) GenerateLite3Parameters();
        else{
            std::cerr << "Not Deafult Robot" << std::endl;
        }
    }
    ~ControlParameters(){}

    /**
     * @brief robot link length
     */
    float body_len_x_, body_len_y_;
    float hip_len_, thigh_len_, shank_len_;

    /**
     * @brief stand height configure
     */
    float pre_height_, stand_height_; 

    /**
     * @brief one leg joint PD gain
     */
    Vec3f swing_leg_kp_, swing_leg_kd_;

    /**
     * @brief joint position limitation
     */
    Vec3f fl_joint_lower_, fl_joint_upper_;

    /**
     * @brief joint velocity limitation
     */
    Vec3f joint_vel_limit_;

    /**
     * @brief joint torque limitation
     */
    Vec3f torque_limit_;
    
    /**
     * @brief stand up duration
     */
    float stand_duration_ = 1.5;

    /**
     * @brief policy path
     */
    std::string common_policy_path_;
    Vec3f common_policy_p_gain_, common_policy_d_gain_;

    // std::string speed_policy_path_;
    // Vec3f speed_policy_p_gain_, speed_policy_d_gain_;

    // std::string tumbler_policy_path_;
    // Vec3f tumbler_policy_p_gain_, tumbler_policy_d_gain_;
};



#endif