#ifndef RAISIM_INTERFACE_HPP_
#define RAISIM_INTERFACE_HPP_

#include "robot_interface.h"
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "raisim/contact/Contact.hpp"

namespace interface{
class RaisimInterface : public RobotInterface
{
private:
    Mat3d QuatToRm(const Vec4d &q){
        double e0 = q(0), e1 = q(1), e2 = q(2), e3 = q(3);

        Mat3d R;
        R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3), 2 * (e1 * e3 + e0 * e2), 
                2 * (e1 * e2 + e0 * e3), 1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
                2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1), 1 - 2 * (e1 * e1 + e2 * e2);
        return R;
    }

    Vec3d QuatToRpy(const Vec4d &q){
        Vec3d rpy;
        double as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
        rpy(2) = std::atan2(2 * (q[1] * q[2] + q[0] * q[3]),
                    q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        rpy(1) = std::asin(as);
        rpy(0) = std::atan2(2 * (q[2] * q[3] + q[0] * q[1]),
                    q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        return rpy;
    }

    void AddSlopeTerrain(){
        double slope_tan = 0.5;
        std::vector<double> height = {0, slope_tan, 2*slope_tan, slope_tan, 0, 
                                        0, slope_tan, 2*slope_tan, slope_tan, 0, 
                                        0, slope_tan, 2*slope_tan, slope_tan, 0, 
                                        0, slope_tan, 2*slope_tan, slope_tan, 0,
                                        0, slope_tan, 2*slope_tan, slope_tan, 0};
        auto heightMap = world_.addHeightMap(5, 5, 10, 10, 4, 0, height);
    }

    Vec3f omega_body_, rpy_, acc_;
    VecXf joint_pos_, joint_vel_, joint_tau_;
    
    std::thread sim_thread_;

public:
    RaisimInterface(const std::string& robot_name, int dof_num=12):RobotInterface(robot_name, dof_num), 
    gyro_nd_(0, 0.0001), rpy_nd_(0, 0.005), acc_nd_(0, 0.0){
        dt_ = 0.001;
        run_cnt_ = 0;
        gravity_ = gravity;
        joint_pos_ = VecXf::Zero(dof_num_);
        joint_tau_ = VecXf::Zero(dof_num_);
        joint_vel_ = VecXf::Zero(dof_num_);

        joint_cmd_ = MatXf::Zero(dof_num_, 5);

        activation_key_ = "";
        urdf_file_ = "";
        std::cout << robot_name << " is using Raisim Simulation" << std::endl;
    }

    ~RaisimInterface(){}

    virtual double GetInterfaceTimeStamp(){
        return run_time_;
    }
    virtual VecXf GetJointPosition() {
        return joint_pos_;
    };
    virtual VecXf GetJointVelocity() {
        return joint_vel_;
    }
    virtual VecXf GetJointTorque() {
        return joint_tau_;
    }
    virtual Vec3f GetImuRpy() {
        return rpy_;
    }
    virtual Vec3f GetImuAcc() {
        return acc_;
    }
    virtual Vec3f GetImuOmega() {
        return omega_body_;
    }
    virtual VecXf GetContactForce() {
        return VecXf::Zero(4);
    }
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input){
        joint_cmd_ = input;
    }

    virtual void Start(){
        sim_thread_ = std::thread(std::bind(&RaisimInterface::Run, this));
    }

    virtual void Stop(){
        start_flag_ = false;
        sim_thread_.join();
    }

    void SetActivationKey(const std::string& ak) {
        activation_key_ = ak;
    }

    void SetUrdfModelPath(const std::string& up){
        urdf_file_ = up;
    }

    double dt_, run_time_;
    double gravity_;
    int run_cnt_;

    raisim::ArticulatedSystem *jueying_;
    raisim::World world_;
    std::shared_ptr<raisim::RaisimServer> server_;

    std::default_random_engine dre_;
    std::normal_distribution<> gyro_nd_, rpy_nd_, acc_nd_;
    Vec3d last_vel_;
    std::string activation_key_, urdf_file_;
    VecXd tau_ff_;

    void ImuStep(){
        Vec4d ori = jueying_->getGeneralizedCoordinate().e().segment(3, 4);
        Mat3d R = QuatToRm(ori);
        Vec3d ori_vel = R.transpose() * jueying_->getGeneralizedVelocity().e().segment(3, 3);
        Vec3d curr_vel = jueying_->getGeneralizedVelocity().e().head(3);

        omega_body_(0) = ori_vel(0) + gyro_nd_(dre_);
        omega_body_(1) = ori_vel(1) + gyro_nd_(dre_);
        omega_body_(2) = ori_vel(2) + gyro_nd_(dre_);

        Vec3d rpy = QuatToRpy(ori);
        rpy_(0) = rpy(0) + rpy_nd_(dre_);
        rpy_(1) = rpy(1) + rpy_nd_(dre_);
        rpy_(2) = rpy(2) + rpy_nd_(dre_); 

        Vec3d acc = (curr_vel - last_vel_) / dt_;
        acc(2) += gravity_;   
        acc = R.transpose() * acc;
        acc_(0) = acc(0) + acc_nd_(dre_);
        acc_(1) = acc(1) + acc_nd_(dre_);
        acc_(2) = acc(2) + acc_nd_(dre_);
        last_vel_ = curr_vel;
    }

    void GetLegDataFeedback(){
        VecXd model_angle = jueying_->getGeneralizedCoordinate().e().tail(dof_num_);
        VecXd model_vel = jueying_->getGeneralizedVelocity().e().tail(dof_num_);
        VecXd model_force = jueying_->getGeneralizedForce().e().tail(dof_num_);

        joint_pos_ = model_angle.cast<float>();
        joint_vel_ = model_vel.cast<float>();
        joint_tau_ = model_force.cast<float>();
    // std::cout << "joint_pos:  " << joint_pos_.transpose() << std::endl;
    }

    void Run(){
        raisim::World::setActivationKey(activation_key_);
        world_.setTimeStep(dt_);
        world_.addGround();
        jueying_ = world_.addArticulatedSystem(urdf_file_);
        jueying_->setName(robot_name_);
        server_ = std::make_shared<raisim::RaisimServer>(&world_);

        VecXd joint_config(jueying_->getGeneralizedCoordinateDim());
        tau_ff_.resize(jueying_->getDOF());
        double yaw0 = 0.0 * M_PI / 2.;
        joint_config << 0, 0, 0.5, 
                    cos(yaw0 / 2), 0, 0, sin(yaw0 / 2), 
                    0, -0.5, 2, 
                    0, -0.5, 2, 
                    0, -0.5, 2, 
                    0, -0.5, 2;
        jueying_->setGeneralizedCoordinate(joint_config);

        std::cout << "********** simulation start **********" << std::endl;
        server_->launchServer();
        server_->focusOn(jueying_);

        while(start_flag_){
            run_time_ = run_cnt_ * dt_;
            ImuStep();
            GetLegDataFeedback();

            VecXf tau_output(dof_num_);
            VecXf kp = joint_cmd_.col(0);
            VecXf goal_joint_pos = joint_cmd_.col(1);
            VecXf kd = joint_cmd_.col(2);
            VecXf goal_joint_vel = joint_cmd_.col(3);
            VecXf tau_ff = joint_cmd_.col(4);

            tau_output = kp.cwiseProduct(goal_joint_pos - joint_pos_)
                        + kd.cwiseProduct(goal_joint_vel - joint_vel_)
                        + tau_ff;

            tau_ff_.setZero();
            tau_ff_.tail(dof_num_) = tau_output.cast<double>();


            jueying_->setGeneralizedForce(tau_ff_);

            if(run_cnt_ % 1000 == 0) {
                std::cout << "time:  " << run_cnt_ / 1000 << " s" << std::endl;
            }

            ++run_cnt_;
            raisim::MSLEEP(dt_*1000*1.0);
            server_->integrateWorldThreadSafe();
        }
        server_->stopRecordingVideo();
        server_->killServer();
        std::cout << "********** simulation end **********" << std::endl;
    }

};

};


#endif