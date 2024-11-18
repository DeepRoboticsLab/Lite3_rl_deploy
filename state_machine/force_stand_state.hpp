/**
 * @file force_stand_state.hpp
 * @brief 
 * @author mazunwang
 * @version 1.0
 * @date 2024-09-15
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef FORCE_STAND_STATE_HPP_
#define FORCE_STAND_STATE_HPP_

#include "state_base.h"

class ForceStandState : public StateBase{
private:
    // VecXf joint_pos_, joint_vel_, joint_tau_;
    Vec3f rpy_, acc_, omg_;
    VecXf init_joint_pos_, init_joint_vel_, current_joint_pos_, current_joint_vel_;
    Mat3f rot_mat_;
    Mat3f fl_jacobian_, fr_jacobian_, hl_jacobian_, hr_jacobian_;
    Vec3f fl_pos_local_, fr_pos_local_, hl_pos_local_, hr_pos_local_;
    Vec3f fl_vel_local_, fr_vel_local_, hl_vel_local_, hr_vel_local_;
    Vec3f fl_pos_global_, fr_pos_global_, hl_pos_global_, hr_pos_global_;
    Vec3f fl_vel_global_, fr_vel_global_, hl_vel_global_, hr_vel_global_;
    Vec3f com_pos_global_, com_vel_global_;
    Vec3f init_com_pos_, init_com_vel_;

    float init_yaw_;
    Vec3f init_rpy_;
    Mat3f init_rot_mat_;
    double time_stamp_record_, run_time_;
    Eigen::Matrix<float, 12, 5> joint_cmd_;
    float cmd_roll_, cmd_pitch_, cmd_yaw_, cmd_height_;
    float input_roll_ = 0, input_pitch_ = 0, input_yaw_ = 0, input_height_ = 0;
    float input_x_ = 0, input_y_ = 0;



    Vec3f LogRot(Mat3f rot){
        float theta;
        float tmp = (rot(0, 0) + rot(1, 1) + rot(2, 2) - 1) / 2;
        if (tmp >= 1.) theta = 0;
        else if (tmp <= -1.) theta = M_PI;
        else theta = acos(tmp);
        Vec3f omega;
        omega << rot(2, 1) - rot(1, 2), rot(0, 2) - rot(2, 0), rot(1, 0) - rot(0, 1);
        if (theta > 10e-5) omega *= theta / (2 * sin(theta)); 
        else omega /= 2;
        return omega;
    }

    void GetProprioceptiveData(){
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        current_joint_pos_ = ri_ptr_->GetJointPosition();
        current_joint_vel_ = ri_ptr_->GetJointVelocity();
        // joint_tau_ = ri_ptr_->GetJointTorque();
        rpy_ = ri_ptr_->GetImuRpy();
        acc_ = ri_ptr_->GetImuAcc();
        omg_ = ri_ptr_->GetImuOmega();
        rot_mat_ = RpyToRm(rpy_);
        GetLegPosLocalAndGlobal();
        cmd_pitch_ = uc_ptr_->GetUserCommand().forward_vel_scale*Deg2Rad(20.);
        cmd_yaw_ = uc_ptr_->GetUserCommand().side_vel_scale*Deg2Rad(15.);
        cmd_roll_ = uc_ptr_->GetUserCommand().turnning_vel_scale*Deg2Rad(10.);
        cmd_height_ = uc_ptr_->GetUserCommand().reserved_scale*0.1;
        cmd_height_ = LimitNumber(cmd_height_, -0.08, 0.05);
        if(fabs(cmd_pitch_) > Deg2Rad(10.) || fabs(cmd_yaw_) > Deg2Rad(10.)){
            cmd_height_ = LimitNumber(cmd_height_, -0.08, -0.05);
        }

        float delta = 0;

        delta = cmd_roll_ - input_roll_;
        delta = LimitNumber(delta, Deg2Rad(0.01));
        input_roll_ = input_roll_ + delta;

        delta = cmd_pitch_ - input_pitch_;
        delta = LimitNumber(delta, Deg2Rad(0.02));
        input_pitch_ = input_pitch_ + delta;

        delta = 0. - input_x_;
        delta = LimitNumber(delta, 3e-5);
        input_x_ = input_x_ + delta;

        delta = 0. - input_y_;
        delta = LimitNumber(delta, 3e-5);
        input_y_ = input_y_ + delta;

        delta = init_rpy_(2) + cmd_yaw_ - input_yaw_;
        delta = LimitNumber(delta, Deg2Rad(0.02));
        input_yaw_ = input_yaw_ + delta;

        delta = cp_ptr_->stand_height_ + cmd_height_ - input_height_;
        delta = LimitNumber(delta, 5e-5);
        input_height_ = input_height_ + delta;
    }

    void RecordJointData(){
        init_joint_pos_ = current_joint_pos_;
        init_joint_vel_ = current_joint_vel_;
        init_com_pos_ = com_pos_global_;
        init_com_vel_ = com_vel_global_;
        time_stamp_record_ = run_time_;
        init_rot_mat_ = rot_mat_;
        init_rpy_ = rpy_;

        input_roll_ = rpy_(0);
        input_pitch_ = rpy_(1);
        input_x_ = init_com_pos_(0);
        input_y_ = init_com_pos_(1);
        input_yaw_ = rpy_(2);
        input_height_ = init_com_pos_(2);
    }

    void GetLegPosLocalAndGlobal(){
        fl_pos_local_ = GetLegPos(current_joint_pos_.segment(0, 3), 0);
        fr_pos_local_ = GetLegPos(current_joint_pos_.segment(3, 3), 1);
        hl_pos_local_ = GetLegPos(current_joint_pos_.segment(6, 3), 2);
        hr_pos_local_ = GetLegPos(current_joint_pos_.segment(9, 3), 3);

        fl_vel_local_ = GetLegVel(current_joint_pos_.segment(0, 3), current_joint_vel_.segment(0, 3), 0);
        fr_vel_local_ = GetLegVel(current_joint_pos_.segment(3, 3), current_joint_vel_.segment(3, 3), 1);
        hl_vel_local_ = GetLegVel(current_joint_pos_.segment(6, 3), current_joint_vel_.segment(6, 3), 2);
        hr_vel_local_ = GetLegVel(current_joint_pos_.segment(9, 3), current_joint_vel_.segment(9, 3), 3);

        fl_pos_global_ = rot_mat_*(fl_pos_local_ + 0.5*Vec3f(cp_ptr_->body_len_x_, cp_ptr_->body_len_y_, 0));
        fr_pos_global_ = rot_mat_*(fr_pos_local_ + 0.5*Vec3f(cp_ptr_->body_len_x_, -cp_ptr_->body_len_y_, 0));
        hl_pos_global_ = rot_mat_*(hl_pos_local_ + 0.5*Vec3f(-cp_ptr_->body_len_x_, cp_ptr_->body_len_y_, 0));
        hr_pos_global_ = rot_mat_*(hr_pos_local_ + 0.5*Vec3f(-cp_ptr_->body_len_x_, -cp_ptr_->body_len_y_, 0));

        Mat3f dot_rot_mat = rot_mat_*CrossVector(omg_);
        fl_vel_global_ = rot_mat_*fl_vel_local_ + dot_rot_mat*fl_pos_local_;
        fr_vel_global_ = rot_mat_*fr_vel_local_ + dot_rot_mat*fr_pos_local_;
        hl_vel_global_ = rot_mat_*hl_vel_local_ + dot_rot_mat*hl_pos_local_;
        hr_vel_global_ = rot_mat_*hr_vel_local_ + dot_rot_mat*hr_pos_local_;

        com_pos_global_ = -(fl_pos_global_+fr_pos_global_+hl_pos_global_+hr_pos_global_)/4.;
        com_vel_global_ = -(fl_vel_global_+fr_vel_global_+hl_vel_global_+hr_vel_global_)/4.;

        fl_jacobian_ = GetLegJacobian(current_joint_pos_.segment(0, 3), 0);
        fr_jacobian_ = GetLegJacobian(current_joint_pos_.segment(3, 3), 1);
        hl_jacobian_ = GetLegJacobian(current_joint_pos_.segment(6, 3), 2);
        hr_jacobian_ = GetLegJacobian(current_joint_pos_.segment(9, 3), 3);
    }

    Vec3f GetLegPos(Vec3f angle, int i){
        float l0, l1, l2;
        float s1, s2, s3;
        float x, y, z, zt;
        l0 = cp_ptr_->hip_len_; l1 = cp_ptr_->thigh_len_; l2 = cp_ptr_->shank_len_;
        if(i%2==0) l0 = -cp_ptr_->hip_len_;
        s1 = angle[0]; s2 = angle[1]; s3 = angle[2];

        x = l1 * sin(s2) + l2 * sin(s2 + s3);
        zt = -(l1 * cos(s2) + l2 * cos(s2 + s3));
        y = zt * sin(s1) - l0 * cos(s1);
        z = zt * cos(s1) + l0 * sin(s1);

        return Vec3f(x, y, z);
    }

    Mat3f GetLegJacobian(Vec3f angle, int leg){
        float x = cp_ptr_->hip_len_, y = -cp_ptr_->thigh_len_, k = -cp_ptr_->shank_len_;
        if(leg == 1 || leg == 3) x = -cp_ptr_->hip_len_;
        float q1 = angle(0), q2 = angle(1), q3 = angle(2);
        float c1 = std::cos(q1), s1 = std::sin(q1);
        float c2 = std::cos(q2), s2 = std::sin(q2);
        float c3 = std::cos(q3), s3 = std::sin(q3);
        Mat3f J;
        J(0, 0) = 0;
        J(0, 1) = -k * (c2 * c3 - s2 * s3) - y * c2;
        J(0, 2) = -k * (c2 * c3 - s2 * s3);
        J(1, 0) = k * (c1 * c2 * c3 - c1 * s2 * s3) - x * s1 + y * c1 * c2;
        J(1, 1) = -k * (c2 * s1 * s3 + c3 * s1 * s2) - y * s1 * s2;
        J(1, 2) = -k * (c2 * s1 * s3 + c3 * s1 * s2);
        J(2, 0) = k * (s1 * s2 * s3 - c2 * c3 * s1) - x * c1 - y * c2 * s1;
        J(2, 1) = -k * (c1 * c2 * s3 + c1 * c3 * s2) - y * c1 * s2;
        J(2, 2) = -k * (c1 * c2 * s3 + c1 * c3 * s2);
        return J;
    }

    Vec3f GetLegVel(Vec3f angle, Vec3f ang_vel, int i){
        float l0, l1, l2;
        float s1, s2, s3;
        float ds1, ds2, ds3;
        float x, y, z, zt;
        float dx, dy, dzt, dz;
        l0 = cp_ptr_->hip_len_; l1 = cp_ptr_->thigh_len_; l2 = cp_ptr_->shank_len_;
        if(i%2==0) l0 = -cp_ptr_->hip_len_;
        s1 = angle[0]; s2 = angle[1]; s3 = angle[2];
        ds1 = ang_vel[0]; ds2 = ang_vel[1]; ds3 = ang_vel[2];

        dx = l1 * cos(s2) * ds2 + l2 * cos(s2 + s3) * (ds2 + ds3);
        zt = -(l1 * cos(s2) + l2 * cos(s2 + s3));
        dzt = l1 * sin(s2) * ds2 + l2 * sin(s2 + s3) * (ds2 + ds3);
        dy = dzt * sin(s1) + zt * cos(s1) * ds1 + l0 * sin(s1) * ds1;
        dz = dzt * cos(s1) - zt * sin(s1) * ds1 + l0 * cos(s1) * ds1;

        return Vec3f(dx, dy, dz);
    }

    void VirtualForceDistribution(){
        Vec3f target_base_pos = Vec3f(input_x_, input_y_, input_height_);
        Vec3f target_base_rpy = Vec3f(input_roll_, input_pitch_, input_yaw_);
        Mat3f target_base_rot = RpyToRm(target_base_rpy);
        Vec3f pos_kp(400, 400, 3500), ori_kp(100, 150, 150);
        Vec3f pos_kd(20, 20, 50), ori_kd(5, 8, 10);
        Vec3f force = pos_kp.cwiseProduct(target_base_pos-com_pos_global_)
                    + pos_kd.cwiseProduct(Vec3f::Zero()-com_vel_global_) + Vec3f(0, 0, 100);
        Vec3f angle_diff = target_base_rpy - rpy_;
        angle_diff = LogRot(target_base_rot*rot_mat_.transpose());
        Vec3f torque = ori_kp.cwiseProduct(angle_diff)
                    + ori_kd.cwiseProduct(Vec3f::Zero()-rot_mat_*omg_);

        Eigen::Matrix<float, 6, 12> A;
        Eigen::Matrix<float, 6, 1> b; 
        Eigen::Matrix<float, 12, 1> F_solution;

        A << Mat3f::Identity(), Mat3f::Identity(), Mat3f::Identity(), Mat3f::Identity(),
            CrossVector(fl_pos_global_), CrossVector(fr_pos_global_), CrossVector(hl_pos_global_), CrossVector(hr_pos_global_);

        b << force, torque;

        F_solution = A.transpose()*(A*A.transpose()).inverse()*b;
        // std::cout << "F_solution: " << F_solution.transpose() << std::endl;
        // std::cout << "b:          " << b.transpose() << std::endl;
        // std::cout << "b_inv:      " << (A*F_solution).transpose() << std::endl;
        for(int leg=0;leg<4;++leg){
            Mat3f leg_jacobian = GetLegJacobian(current_joint_pos_.segment(3*leg, 3), leg);
            Vec3f leg_force = F_solution.segment(3*leg, 3);
            Vec3f leg_torque = -leg_jacobian.transpose()*rot_mat_.transpose()*leg_force;
            joint_cmd_.block(3*leg, 4, 3, 1) = leg_torque;

            // Vec3f leg_vel_cal = GetLegVel(current_joint_pos_.segment(3*leg, 3), current_joint_vel_.segment(3*leg, 3), leg);
            // Vec3f leg_vel_jac = leg_jacobian*current_joint_vel_.segment(3*leg, 3);
            // std::cout << leg << " : " << leg_vel_cal.transpose() << " | " << leg_vel_jac.transpose() << std::endl;
        }
        // std::cout << "joint_cmd: " << joint_cmd_ << std::endl;
    }


public:
    ForceStandState(const RobotType& robot_type, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr){
    }
    ~ForceStandState(){}

    virtual void OnEnter() {
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::ForceStand);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
        GetProprioceptiveData();
        RecordJointData();
    };
    virtual void OnExit() {

    }
    virtual void Run() {
        GetProprioceptiveData();
        joint_cmd_.setZero();
        VirtualForceDistribution();
        ri_ptr_->SetJointCommand(joint_cmd_);
    }
    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) return true;
        Vec3f rpy = ri_ptr_->GetImuRpy();
        if(rpy(0) > 30./180*M_PI || rpy(1) > 45./180*M_PI){
            std::cout << "posture value: " << 180./M_PI*rpy.transpose() << std::endl;
            return true;
        }
        return false;
    }
    virtual StateName GetNextStateName() {
        return StateName::kForceStand;
    }
};

#endif