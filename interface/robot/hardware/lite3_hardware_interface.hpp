#ifndef LITE3_HARDWARE_INTERFACE_HPP_
#define LITE3_HARDWARE_INTERFACE_HPP_

#include "robot_interface.h"
#include "lite3_types.h"
#include "receiver.h"
#include "sender.h"

using namespace lite3;

class Lite3HardwareInterface : public RobotInterface
{
private:
    RobotData* robot_data_=nullptr;
    RobotCmd robot_joint_cmd_{};
    Receiver* receiver_ = nullptr;
    Sender* sender_ = nullptr;

    Vec3f omega_body_, rpy_, acc_;
    VecXf joint_pos_, joint_vel_, joint_tau_;
    std::thread hw_thread_;
public:
    Lite3HardwareInterface(const std::string& robot_name, 
                        int local_port=43897, 
                        std::string robot_ip="192.168.2.1",
                        int robot_port=43893):RobotInterface(robot_name, 12){
        std::cout << robot_name << " is using Lite3 Hardware Interface" << std::endl;
        receiver_ = new Receiver(local_port);
        sender_ = new Sender(robot_ip, robot_port);
        sender_->RobotStateInit();
    }
    ~Lite3HardwareInterface(){}

    virtual void Start(){
        receiver_->StartWork();                        
        robot_data_ = &(receiver_->GetState());
        if (sender_ != nullptr)
            sender_->ControlGet(2);
    }

    virtual void Stop(){
        if(sender_ != nullptr){
            sender_->ControlGet(1);
        }
    }

    virtual float GetInterfaceTimeStamp(){
        return robot_data_->tick*0.001;
    }
    virtual VecXf GetJointPosition() {
        joint_pos_ = VecXf::Zero(dof_num_);
        for(int i=0;i<dof_num_;++i){
            joint_pos_(i) = robot_data_->joint_data.joint_data[i].position;
        }
        return joint_pos_;
    };
    virtual VecXf GetJointVelocity() {
        joint_vel_ = VecXf::Zero(dof_num_);
        for(int i=0;i<dof_num_;++i){
            joint_vel_(i) = robot_data_->joint_data.joint_data[i].velocity;
        }
        return joint_vel_;
    }
    virtual VecXf GetJointTorque() {
        joint_tau_ = VecXf::Zero(dof_num_);
        for(int i=0;i<dof_num_;++i){
            joint_tau_(i) = robot_data_->joint_data.joint_data[i].torque;
        }
        return joint_tau_;
    }
    virtual Vec3f GetImuRpy() {
        rpy_ << robot_data_->imu.roll/180.*M_PI, robot_data_->imu.pitch/180.*M_PI, robot_data_->imu.yaw/180.*M_PI;
        return rpy_;
    }
    virtual Vec3f GetImuAcc() {
        acc_ << robot_data_->imu.acc_x, robot_data_->imu.acc_y, robot_data_->imu.acc_z;
        return acc_;
    }
    virtual Vec3f GetImuOmega() {
        omega_body_ << robot_data_->imu.omega_x, robot_data_->imu.omega_y, robot_data_->imu.omega_z;
        return omega_body_;
    }
    virtual VecXf GetContactForce() {
        return VecXf::Zero(4);
    }
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input){
        for(int i=0;i<dof_num_;++i){
            robot_joint_cmd_.joint_cmd[i].kp       = input(i, 0);
            robot_joint_cmd_.joint_cmd[i].position = input(i, 1);
            robot_joint_cmd_.joint_cmd[i].kd       = input(i, 2);
            robot_joint_cmd_.joint_cmd[i].velocity = input(i, 3);
            robot_joint_cmd_.joint_cmd[i].torque   = input(i, 4);
        }
        joint_cmd_ = input;
        sender_->SendCmd(robot_joint_cmd_);
    }
};



#endif