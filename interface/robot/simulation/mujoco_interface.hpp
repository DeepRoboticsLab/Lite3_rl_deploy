/**
 * @file mujoco_interface.hpp
 * @brief simulation in mujoco
 * @author mayuxuan
 * @version 1.0
 * @date 2025-05-08
 * @copyright Copyright (c) 2025 DeepRobotics
 */


#ifndef MUJOCO_INTERFACE_HPP_
#define MUJOCO_INTERFACE_HPP_

#include "robot_interface.h"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <string>
#include <thread>
#include <iostream>
#include <cstring>
#include <random>


#include <atomic>
#include <mutex>

namespace interface {

class MujocoInterface : public RobotInterface {
private:
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;

    std::string xml_path_;
    mjvScene scene_;
    mjrContext context_;
    mjvCamera camera_;
    mjvOption opt_;
    GLFWwindow* window_ = nullptr;

    std::thread sim_thread_;

    Vec3d omega_body_, rpy_, acc_;

    VecXd joint_pos_, joint_vel_, joint_tau_;


    double dt_ = 0.001;
    double run_time_ = 0.0;
    int run_cnt_ = 0;
    VecXd tau_ff_;

    int render_interval_ = 10;

    std::default_random_engine dre_;
    std::normal_distribution<> gyro_nd_{0, 0.0001}, rpy_nd_{0, 0.005}, acc_nd_{0, 0.0};

public:
    MujocoInterface(const std::string& robot_name,
                const std::string& xml_path,
                int dof_num = 12)
        : RobotInterface("MujocoSim", dof_num), xml_path_(xml_path) {

        joint_pos_ = VecXd::Zero(dof_num_);
        joint_tau_ = VecXd::Zero(dof_num_);
        joint_vel_ = VecXd::Zero(dof_num_);
        joint_cmd_ = MatXf::Zero(dof_num_, 5);

        

        std::cout << "[MuJoCoInterface] Loading model: " << xml_path_ << std::endl;
        char error[1000] = "";
        model_ = mj_loadXML(xml_path_.c_str(), 0, error, 1000);
        if (!model_) {
            std::cerr << "[ERROR] Failed to load MuJoCo model: " << error << std::endl;
            exit(1);
        }
        data_ = mj_makeData(model_);

        // 可视化初始化
        // mjv_defaultCamera(&camera_);
        // mjv_defaultOption(&opt_);
        // mjv_defaultScene(&scene_);
        // mjr_defaultContext(&context_);

        // if (!glfwInit()) {
        //     std::cerr << "[ERROR] Could not initialize GLFW" << std::endl;
        //     exit(1);
        // }

        // window_ = glfwCreateWindow(1200, 900, "MuJoCo Simulation", NULL, NULL);
        // if (!window_) {
        //     std::cerr << "[ERROR] Could not create GLFW window" << std::endl;
        //     glfwTerminate();
        //     exit(1);
        // }

        // glfwMakeContextCurrent(window_);
        // mjv_makeScene(model_, &scene_, 2000);
        // mjr_makeContext(model_, &context_, mjFONTSCALE_150);

        std::cout << "[MuJoCoInterface] Model loaded successfully. DOF: " << model_->nu << std::endl;

        camera_.type = mjCAMERA_TRACKING;
        camera_.trackbodyid = mj_name2id(model_, mjOBJ_BODY, "TORSO");  // “base”为MJCF中主体名
        // camera_.lookat[0] = 0.0;
        // camera_.lookat[1] = 0.0;
        // camera_.lookat[2] = 1.0;
        camera_.distance = 4.0;
        camera_.azimuth = 90.0;
        camera_.elevation = -30.0;
    }

    ~MujocoInterface() {
        mj_deleteData(data_);
        mj_deleteModel(model_);
        mjv_freeScene(&scene_);
        mjr_freeContext(&context_);
        if (window_) glfwDestroyWindow(window_);
        glfwTerminate();
    }

    virtual double GetInterfaceTimeStamp() override { return run_time_; }

    // virtual VecXf GetJointPosition() override { return joint_pos_; }
    // virtual VecXf GetJointVelocity() override { return joint_vel_; }
    // virtual VecXf GetJointTorque() override { return joint_tau_; }
    // virtual Vec3f GetImuRpy() override { return rpy_; }
    // virtual Vec3f GetImuAcc() override { return acc_; }
    // virtual Vec3f GetImuOmega() override { return omega_body_; }
    // virtual VecXf GetContactForce() override { return VecXf::Zero(4); }

    virtual VecXf GetJointPosition() override { return joint_pos_.cast<float>(); }
    virtual VecXf GetJointVelocity() override { return joint_vel_.cast<float>(); }
    virtual VecXf GetJointTorque() override { return joint_tau_.cast<float>(); }
    virtual Vec3f GetImuRpy() override { return rpy_.cast<float>(); }
    virtual Vec3f GetImuAcc() override { return acc_.cast<float>(); }
    virtual Vec3f GetImuOmega() override { return omega_body_.cast<float>(); }
    virtual VecXf GetContactForce() override { return VecXf::Zero(4); }



    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) override {
        joint_cmd_ = input;
    }

    virtual void Start() override {
        start_flag_ = true;
        sim_thread_ = std::thread(std::bind(&MujocoInterface::Run, this));
    }

    virtual void Stop() override {
        start_flag_ = false;
        sim_thread_.join();
    }

private:
    void Run() {


        // 可视化初始化
        // mjv_defaultCamera(&camera_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scene_);
        mjr_defaultContext(&context_);

        if (!glfwInit()) {
            std::cerr << "[ERROR] Could not initialize GLFW" << std::endl;
            exit(1);
        }

        window_ = glfwCreateWindow(1200, 900, "MuJoCo Simulation", NULL, NULL);
        if (!window_) {
            std::cerr << "[ERROR] Could not create GLFW window" << std::endl;
            glfwTerminate();
            exit(1);
        }

        glfwMakeContextCurrent(window_);
        mjv_makeScene(model_, &scene_, 2000);
        mjr_makeContext(model_, &context_, mjFONTSCALE_150);

        glfwMakeContextCurrent(window_);

        glfwSwapInterval(1);

        mj_forward(model_, data_);
        
        
        
        while (start_flag_ && !glfwWindowShouldClose(window_))  {
            run_time_ = run_cnt_ * dt_;

            UpdateImu();
            UpdateJointState();
            ApplyControl();

            mj_step(model_, data_);

            if (run_cnt_ % render_interval_ == 0) {
                Render();
            }        
            // std::cout << "Rendered frame " << run_cnt_ << std::endl;

            ++run_cnt_;
            std::this_thread::sleep_for(std::chrono::microseconds(int(dt_ * 1e6)));
        }
    }

    void UpdateImu() {

        double* q = data_->qpos + 3;
        double qw = q[0], qx = q[1], qy = q[2], qz = q[3];

        double roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
        double pitch = asin(2 * (qw * qy - qz * qx));
        double yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

        rpy_ << roll + rpy_nd_(dre_), pitch + rpy_nd_(dre_), yaw + rpy_nd_(dre_);

        acc_ = Eigen::Map<Vec3d>(data_->sensordata + 16);

        omega_body_ = Eigen::Map<Vec3d>(data_->qvel + 3) + Vec3d(gyro_nd_(dre_), gyro_nd_(dre_), gyro_nd_(dre_));

        // std::cout << "[IMU] RPY: " << rpy_.transpose()
        //       << " | Omega: " << omega_body_.transpose()
        //       << " | Acc: " << acc_.transpose() << std::endl;
        
        
    }

    void UpdateJointState() {
        joint_pos_ = Eigen::Map<VecXd>(data_->qpos + 7, dof_num_);
        joint_vel_ = Eigen::Map<VecXd>(data_->qvel + 6, dof_num_);
        joint_tau_ = Eigen::Map<VecXd>(data_->ctrl, dof_num_);

        // std::cout << "[JointState] pos[0:3]: " << joint_pos_.head(3).transpose()
        //       << " | vel[0:3]: " << joint_vel_.head(3).transpose()
        //       << " | tau[0:3]: " << joint_tau_.head(3).transpose() << std::endl;
        
    }

    void ApplyControl() {
        auto kp = joint_cmd_.col(0).cast<double>();
        auto q_des = joint_cmd_.col(1).cast<double>();
        auto kd = joint_cmd_.col(2).cast<double>();
        auto dq_des = joint_cmd_.col(3).cast<double>();
        auto tau_ff = joint_cmd_.col(4).cast<double>();

        VecXd tau_out = kp.cwiseProduct(q_des - joint_pos_)
                      + kd.cwiseProduct(dq_des - joint_vel_)
                      + tau_ff;
        
        // std::cout << "[ApplyCtrl] tau_out[0:3]: " << tau_out.head(3).transpose()
        //       << " | q_des[0:3]: " << q_des.head(3).transpose()
        //       << " | q[0:3]: " << joint_pos_.head(3).transpose() << std::endl;                  

        // VecXd tau_out = joint_cmd_.col(0).cwiseProduct(joint_cmd_.col(1) - joint_pos_)
        //               + joint_cmd_.col(2).cwiseProduct(joint_cmd_.col(3) - joint_vel_)
        //               + joint_cmd_.col(4);
        Eigen::Map<VecXd>(data_->ctrl, dof_num_) = tau_out;
    }

    void Render() {
        mjv_updateScene(model_, data_, &opt_, nullptr, &camera_, mjCAT_ALL, &scene_);
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
        mjr_render(viewport, &scene_, &context_);
        glfwSwapBuffers(window_);
        glfwPollEvents();
    }
};

}  // namespace interface

#endif  // MUJOCO_INTERFACE_HPP_