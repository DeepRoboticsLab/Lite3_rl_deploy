/**
 * @file robot_interface.h
 * @brief this file is for applying robot's control interface
 * @author mazunwang
 * @version 1.0
 * @date 2024-04-11
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef ROBOT_INTERFACE_H_
#define ROBOT_INTERFACE_H_

#include "common_types.h"
#include <atomic>

using namespace types;

namespace interface{
class RobotInterface{
private:
    /* data */
public:
    /**
     * @brief Construct a new Robot Interface object
     * @param  robot_name       robot name
     * @param  dof_num          The number of degrees of freedom of the robot.
     */
    RobotInterface(const std::string& robot_name, int dof_num):robot_name_(robot_name), dof_num_(dof_num){
        start_flag_ = true;
    }
    virtual ~RobotInterface(){};

    std::string robot_name_;
    const int dof_num_;
    Eigen::Matrix<float, Eigen::Dynamic, 5> joint_cmd_;
    std::atomic<bool> start_flag_;

    /**
     * @brief Start to get robot's state and send control command
     */
    virtual void Start() = 0;

    /**
     * @brief Stop to control the robot
     */
    virtual void Stop() = 0;

    /**
     * @brief Get the time stamp of the robot
     * @return double        time stamp
     */
    virtual double GetInterfaceTimeStamp() = 0;

    /**
     * @brief Get the joint position of the robot
     * @return VecXf        joint position vector
     */
    virtual VecXf GetJointPosition() = 0;

    /**
     * @brief Get the joint velocity of the robot
     * @return VecXf        joint velocity vector
     */
    virtual VecXf GetJointVelocity() = 0;

    /**
     * @brief Get the joint torque of the robot
     * @return VecXf        joint torque vector
     */
    virtual VecXf GetJointTorque() = 0;

    /**
     * @brief Get the roll-pitch-yaw angle of the robot base
     * @return Vec3f        roll-pitch-yaw(unit rad)
     */
    virtual Vec3f GetImuRpy() = 0;

    /**
     * @brief Get the accleration of the robot base
     * @return Vec3f        accleration(unit m*s^-2)
     */
    virtual Vec3f GetImuAcc() = 0;

    /**
     * @brief Get the angular velocity of robot base
     * @return Vec3f        angular velocity in body coordinate(unit rad/s)
     */
    virtual Vec3f GetImuOmega() = 0;

    /**
     * @brief Set the joint command in standard form
     *                  torque = kp * (qDes - q) + kd * (vDes - v) + tff
     * @param  input       a dof_num*5 matrix and each column represent kp, goal_angle_pos, kd, goal_vel, torque_feedforward
     */
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) = 0;

    virtual MatXf GetJointCommand(){
        return joint_cmd_;
    }

    /**
     * @brief Get the contact force if robot have any force sensor
     * @return VecXf        force vector
     */
    virtual VecXf GetContactForce() = 0;

    /**
     * @brief Get the motor temperture 
     * @return VecXf        motor temperture
     */
    virtual VecXf GetMotorTemperture(){
        return VecXf::Zero(dof_num_);
    }

    /**
     * @brief Get the driver temperture 
     * @return VecXf        driver temperture
     */
    virtual VecXf GetDriverTemperture(){
        return VecXf::Zero(dof_num_);
    }
};


};//namespace interface

#endif