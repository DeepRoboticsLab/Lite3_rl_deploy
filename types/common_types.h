
#ifndef COMMON_TYPES_H_
#define COMMON_TYPES_H_

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <vector>
#include <deque>
#include <map>
#include <cmath>
#include <memory>
#include <thread>
#include <sys/timerfd.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/epoll.h>


namespace types{
    using Vec3f = Eigen::Vector3f;
    using Vec3d = Eigen::Vector3d;
    using Vec4f = Eigen::Vector4f;
    using Vec4d = Eigen::Vector4d;
    using VecXf = Eigen::VectorXf;
    using VecXd = Eigen::VectorXd;

    using Mat3f = Eigen::Matrix3f;
    using Mat3d = Eigen::Matrix3d;
    using MatXf = Eigen::MatrixXf;
    using MatXd = Eigen::MatrixXd;

    const float gravity = 9.815;

    struct RobotBasicState{
        Vec3f base_rpy;
        Vec3f projected_gravity;
        Vec4f base_quat;
        Mat3f base_rot_mat;
        Vec3f base_omega;
        Vec3f base_acc;
        Vec3f cmd_vel_normlized; //vel_x, vel_y, turnning_vel
        VecXf joint_pos;
        VecXf joint_vel;
        VecXf joint_tau;
    };

    struct RobotAction{
        VecXf goal_joint_pos;
        VecXf goal_joint_vel;
        VecXf kp;
        VecXf kd;
        VecXf tau_ff;

        MatXf ConvertToMat(){
            MatXf res(goal_joint_pos.rows(), 5);
            res.col(0) = kp; res.col(1) = goal_joint_pos;
            res.col(2) = kd; res.col(3) = goal_joint_vel;
            res.col(4) = tau_ff;
            return res;
        }
    };
    
    
    struct UserCommand{
        bool soft_stop_flag;
        int target_mode;
        int target_gait;
        float forward_vel_scale;
        float side_vel_scale;
        float turnning_vel_scale;
    };

    struct MotionStateFeedback{
        int current_state;
        int current_gait;
        int last_state;
        std::array<float, 3> current_vel;
        std::array<float, 3> goal_vel;
        std::array<float, 3> max_vel; 

        MotionStateFeedback(){
            memset(this, 0, sizeof(MotionStateFeedback));
        }

        void UpdateCurrentState(int state){
            if(state != current_state){
                last_state = current_state;
                current_state = state;
            }
        }
    };
};

#endif