/**
 * @file lite3_controller.hpp
 * @brief High-level wrapper API for Lite3 robot control
 * @author HaiHa
 * @version 1.0
 * @date 2025-10-23
 *
 * @copyright Copyright (c) 2024  DeepRobotics
 *
 * This file provides a simplified, user-friendly API for controlling the Lite3
 * quadruped robot. It abstracts away the complexity of state machines, interfaces,
 * and threading, making it easy to develop new algorithms and policies.
 */

#pragma once

#include "common_types.h"
#include "robot_interface.h"
#include "user_command_interface.h"
#include <functional>
#include <memory>
#include <string>
#include <atomic>
#include <thread>
#include <chrono>

namespace lite3_api {

using namespace types;
using namespace interface;

/**
 * @brief Callback function type for custom control policies
 * @param state Current robot state (IMU + joints)
 * @return RobotAction to be executed
 */
using PolicyCallback = std::function<RobotAction(const RobotBasicState&)>;

/**
 * @brief Control modes for the robot
 */
enum class ControlMode {
    IDLE,               ///< Joints unpowered
    POSITION_CONTROL,   ///< Simple position control to target pose
    RL_CONTROL,         ///< Run ONNX-based RL policy
    CUSTOM_CALLBACK     ///< Use custom user-defined policy callback
};

/**
 * @brief Configuration for the controller
 */
struct ControllerConfig {
    float control_frequency = 1000.0f;  ///< Base control loop frequency (Hz)
    float policy_frequency = 83.0f;     ///< Policy execution frequency (Hz)
    float max_roll_deg = 30.0f;         ///< Maximum roll angle for safety (degrees)
    float max_pitch_deg = 45.0f;        ///< Maximum pitch angle for safety (degrees)
    bool enable_logging = false;        ///< Enable data logging
    std::string log_file = "lite3_log.csv";  ///< Log file path
    VecXf default_pose = (Eigen::VectorXf(12) << 0, -0.8, 1.6, 0, -0.8, 1.6,
                                                   0, -0.8, 1.6, 0, -0.8, 1.6).finished();
    VecXf default_kp = VecXf::Constant(12, 30.0f);  ///< Default proportional gains
    VecXf default_kd = VecXf::Constant(12, 1.0f);   ///< Default derivative gains
};

/**
 * @brief Main controller class for Lite3 robot
 *
 * This class provides a high-level API for controlling the Lite3 quadruped robot.
 * It handles:
 * - Automatic detection of simulation vs hardware
 * - State machine management
 * - Safety checks
 * - Threading for control loops
 * - Data logging and monitoring
 *
 * Example usage:
 * @code
 * Lite3Controller robot(true);  // true = simulation mode
 * robot.standUp();
 * robot.setVelocity(0.5, 0.0, 0.0);  // Walk forward
 * robot.loadONNXPolicy("policy.onnx");
 * robot.run(ControlMode::RL_CONTROL);
 * @endcode
 */
class Lite3Controller {
public:
    /**
     * @brief Simple constructor - auto-detects simulation vs hardware
     * @param use_sim  If true, uses PyBullet simulation. If false, uses hardware
     * @param config  Optional configuration parameters
     */
    Lite3Controller(bool use_sim = true, const ControllerConfig& config = ControllerConfig());

    /**
     * @brief Advanced constructor with custom interfaces
     * @param robot_interface  Custom robot interface implementation
     * @param user_interface   Custom user command interface (optional)
     * @param config  Optional configuration parameters
     */
    Lite3Controller(
        std::shared_ptr<RobotInterface> robot_interface,
        std::shared_ptr<UserCommandInterface> user_interface = nullptr,
        const ControllerConfig& config = ControllerConfig()
    );

    /**
     * @brief Destructor - ensures proper cleanup
     */
    ~Lite3Controller();

    // ========== Basic Control Functions ==========

    /**
     * @brief Initialize and start the robot interfaces
     * Call this before any other control commands
     */
    void initialize();

    /**
     * @brief Stand up to default position with smooth interpolation
     * @param duration  Time to reach standing position (seconds)
     * @param blocking  If true, waits until standing is complete
     */
    void standUp(float duration = 2.0f, bool blocking = true);

    /**
     * @brief Set target velocity command (normalized)
     * @param vx  Forward velocity [-1.0, 1.0] (1.0 = max forward speed)
     * @param vy  Lateral velocity [-1.0, 1.0] (1.0 = max lateral speed)
     * @param vyaw  Yaw rate [-1.0, 1.0] (1.0 = max rotation speed)
     */
    void setVelocity(float vx, float vy, float vyaw);

    /**
     * @brief Emergency stop - immediately enable joint damping
     */
    void emergencyStop();

    /**
     * @brief Check if robot is in safe state (orientation within limits)
     * @return true if safe, false if safety limits exceeded
     */
    bool isSafe() const;

    /**
     * @brief Get current control mode
     */
    ControlMode getCurrentMode() const;

    // ========== State Reading Functions ==========

    /**
     * @brief Get current robot state (IMU + joints + commands)
     * @return Complete robot state structure
     */
    RobotBasicState getState() const;

    /**
     * @brief Get joint positions (12-DOF)
     * @return Vector of joint positions [rad]
     */
    VecXf getJointPositions() const;

    /**
     * @brief Get joint velocities (12-DOF)
     * @return Vector of joint velocities [rad/s]
     */
    VecXf getJointVelocities() const;

    /**
     * @brief Get joint torques (12-DOF)
     * @return Vector of joint torques [Nm]
     */
    VecXf getJointTorques() const;

    /**
     * @brief Get base orientation (roll, pitch, yaw)
     * @return Vector [roll, pitch, yaw] in radians
     */
    Vec3f getOrientation() const;

    /**
     * @brief Get base angular velocity in body frame
     * @return Vector [wx, wy, wz] in rad/s
     */
    Vec3f getAngularVelocity() const;

    /**
     * @brief Get base acceleration in body frame
     * @return Vector [ax, ay, az] in m/s^2
     */
    Vec3f getAcceleration() const;

    /**
     * @brief Get current command velocities
     * @return Vector [vx, vy, vyaw] normalized [-1, 1]
     */
    Vec3f getCommandVelocities() const;

    /**
     * @brief Get timestamp of current state (seconds)
     */
    double getTimestamp() const;

    // ========== Advanced Control Functions ==========

    /**
     * @brief Send raw joint commands (PD control with feedforward)
     * @param target_pos  Target joint positions [rad]
     * @param kp  Proportional gains
     * @param kd  Derivative gains
     * @param target_vel  Target joint velocities [rad/s] (default: zero)
     * @param tau_ff  Feedforward torques [Nm] (default: zero)
     */
    void setJointCommand(
        const VecXf& target_pos,
        const VecXf& kp,
        const VecXf& kd,
        const VecXf& target_vel = VecXf::Zero(12),
        const VecXf& tau_ff = VecXf::Zero(12)
    );

    /**
     * @brief Set a custom control policy via callback function
     *
     * The callback will be called at the specified frequency with the current
     * robot state, and should return the desired action.
     *
     * @param callback  Function that takes RobotBasicState and returns RobotAction
     * @param frequency  Control frequency in Hz (default: 83Hz)
     *
     * Example:
     * @code
     * auto my_policy = [](const RobotBasicState& state) {
     *     RobotAction action;
     *     // ... compute action based on state
     *     return action;
     * };
     * robot.setCustomPolicy(my_policy, 100.0f);
     * @endcode
     */
    void setCustomPolicy(PolicyCallback callback, float frequency = 83.0f);

    /**
     * @brief Load and configure an ONNX policy for RL control
     * @param model_path  Path to .onnx model file
     * @param obs_dim  Observation dimension (default: 45)
     * @param act_dim  Action dimension (default: 12)
     */
    void loadONNXPolicy(const std::string& model_path, int obs_dim = 45, int act_dim = 12);

    /**
     * @brief Start the control loop (blocking)
     *
     * This function will run the control loop in the current thread and block
     * until stop() is called from another thread.
     *
     * @param mode  Control mode to use
     */
    void run(ControlMode mode = ControlMode::RL_CONTROL);

    /**
     * @brief Start the control loop in background thread (non-blocking)
     *
     * This allows the main thread to continue and perform other tasks while
     * the control loop runs in the background.
     *
     * @param mode  Control mode to use
     */
    void runAsync(ControlMode mode = ControlMode::RL_CONTROL);

    /**
     * @brief Stop the control loop
     *
     * If running async, this will wait for the background thread to finish.
     */
    void stop();

    /**
     * @brief Check if control loop is currently running
     */
    bool isRunning() const;

    // ========== Configuration Functions ==========

    /**
     * @brief Set default joint positions for standing pose
     * @param default_pos  12-DOF joint position vector [rad]
     */
    void setDefaultPose(const VecXf& default_pos);

    /**
     * @brief Set default PD gains
     * @param kp  Proportional gains (12-DOF)
     * @param kd  Derivative gains (12-DOF)
     */
    void setDefaultGains(const VecXf& kp, const VecXf& kd);

    /**
     * @brief Set safety limits (roll/pitch angles)
     * @param max_roll_deg  Maximum roll angle in degrees
     * @param max_pitch_deg  Maximum pitch angle in degrees
     */
    void setSafetyLimits(float max_roll_deg, float max_pitch_deg);

    /**
     * @brief Enable or disable data logging
     * @param enable  true to enable logging
     * @param log_file  Path to log file (empty = use default)
     */
    void setLogging(bool enable, const std::string& log_file = "");

    /**
     * @brief Set control loop frequency
     * @param frequency  Frequency in Hz
     */
    void setControlFrequency(float frequency);

    /**
     * @brief Set policy execution frequency
     * @param frequency  Frequency in Hz
     */
    void setPolicyFrequency(float frequency);

    /**
     * @brief Get current configuration
     */
    const ControllerConfig& getConfig() const;

    // ========== Utility Functions ==========

    /**
     * @brief Wait for robot to reach target pose (within tolerance)
     * @param target_pos  Target joint positions
     * @param tolerance  Position tolerance [rad] (default: 0.05)
     * @param timeout  Maximum wait time [s] (default: 5.0)
     * @return true if reached, false if timeout
     */
    bool waitForPose(const VecXf& target_pos, float tolerance = 0.05f, float timeout = 5.0f);

    /**
     * @brief Get statistics about control loop performance
     * @return String with timing statistics
     */
    std::string getPerformanceStats() const;

    /**
     * @brief Reset the controller to initial state
     */
    void reset();

private:
    // ========== Private Implementation ==========
    struct Impl;
    std::unique_ptr<Impl> pimpl_;

    // Control loop thread function
    void controlLoop();

    // Safety check implementation
    bool checkSafety() const;

    // Update robot state from interfaces
    void updateState();

    // Execute current control mode
    void executeControl();

    // Log data if logging is enabled
    void logData();

    // Stand up trajectory generation
    VecXf interpolatePose(const VecXf& start, const VecXf& end, float t);
};

} // namespace lite3_api
