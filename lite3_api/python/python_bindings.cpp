/**
 * @file python_bindings.cpp
 * @brief Python bindings for Lite3 API using pybind11
 * @author HaiHa
 * @version 1.0
 * @date 2025-10-23
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include "lite3_controller.hpp"

namespace py = pybind11;
using namespace lite3_api;
using namespace types;

PYBIND11_MODULE(pylite3, m) {
    m.doc() = R"pbdoc(
        PyLite3 - Python bindings for Lite3 Quadruped Robot Control

        This module provides a high-level Python interface for controlling
        the DeepRobotics Lite3 quadruped robot in both simulation and hardware.

        Example:
            import pylite3
            import numpy as np

            # Create controller
            robot = pylite3.Lite3Controller(use_sim=True)

            # Stand up
            robot.stand_up()

            # Walk forward
            robot.set_velocity(0.5, 0.0, 0.0)
            robot.run(pylite3.ControlMode.POSITION_CONTROL)
    )pbdoc";

    // ========== Enums ==========

    py::enum_<ControlMode>(m, "ControlMode", "Robot control modes")
        .value("IDLE", ControlMode::IDLE, "Joints unpowered")
        .value("POSITION_CONTROL", ControlMode::POSITION_CONTROL, "Simple position control")
        .value("RL_CONTROL", ControlMode::RL_CONTROL, "RL policy control (ONNX)")
        .value("CUSTOM_CALLBACK", ControlMode::CUSTOM_CALLBACK, "Custom policy callback")
        .export_values();

    // ========== Structs ==========

    py::class_<RobotBasicState>(m, "RobotBasicState", "Complete robot state information")
        .def(py::init<>())
        .def_readonly("base_rpy", &RobotBasicState::base_rpy,
                     "Base orientation (roll, pitch, yaw) in radians")
        .def_readonly("projected_gravity", &RobotBasicState::projected_gravity,
                     "Projected gravity vector in body frame")
        .def_readonly("base_quat", &RobotBasicState::base_quat,
                     "Base orientation as quaternion")
        .def_readonly("base_rot_mat", &RobotBasicState::base_rot_mat,
                     "Base rotation matrix")
        .def_readonly("base_omega", &RobotBasicState::base_omega,
                     "Base angular velocity [rad/s]")
        .def_readonly("base_acc", &RobotBasicState::base_acc,
                     "Base acceleration [m/s^2]")
        .def_readonly("cmd_vel_normlized", &RobotBasicState::cmd_vel_normlized,
                     "Command velocity (normalized [-1, 1])")
        .def_readonly("joint_pos", &RobotBasicState::joint_pos,
                     "Joint positions [rad] (12-DOF)")
        .def_readonly("joint_vel", &RobotBasicState::joint_vel,
                     "Joint velocities [rad/s] (12-DOF)")
        .def_readonly("joint_tau", &RobotBasicState::joint_tau,
                     "Joint torques [Nm] (12-DOF)")
        .def("__repr__", [](const RobotBasicState& s) {
            return "<RobotBasicState with 12 DOF>";
        });

    py::class_<RobotAction>(m, "RobotAction", "Robot action command")
        .def(py::init<>())
        .def_readwrite("goal_joint_pos", &RobotAction::goal_joint_pos,
                      "Target joint positions [rad]")
        .def_readwrite("goal_joint_vel", &RobotAction::goal_joint_vel,
                      "Target joint velocities [rad/s]")
        .def_readwrite("kp", &RobotAction::kp,
                      "Proportional gains")
        .def_readwrite("kd", &RobotAction::kd,
                      "Derivative gains")
        .def_readwrite("tau_ff", &RobotAction::tau_ff,
                      "Feedforward torques [Nm]")
        .def("__repr__", [](const RobotAction& a) {
            return "<RobotAction with 12 DOF>";
        });

    py::class_<ControllerConfig>(m, "ControllerConfig", "Controller configuration parameters")
        .def(py::init<>())
        .def_readwrite("control_frequency", &ControllerConfig::control_frequency,
                      "Base control loop frequency [Hz]")
        .def_readwrite("policy_frequency", &ControllerConfig::policy_frequency,
                      "Policy execution frequency [Hz]")
        .def_readwrite("max_roll_deg", &ControllerConfig::max_roll_deg,
                      "Maximum roll angle for safety [degrees]")
        .def_readwrite("max_pitch_deg", &ControllerConfig::max_pitch_deg,
                      "Maximum pitch angle for safety [degrees]")
        .def_readwrite("enable_logging", &ControllerConfig::enable_logging,
                      "Enable data logging")
        .def_readwrite("log_file", &ControllerConfig::log_file,
                      "Log file path")
        .def_readwrite("default_pose", &ControllerConfig::default_pose,
                      "Default standing pose (12-DOF)")
        .def_readwrite("default_kp", &ControllerConfig::default_kp,
                      "Default proportional gains")
        .def_readwrite("default_kd", &ControllerConfig::default_kd,
                      "Default derivative gains")
        .def("__repr__", [](const ControllerConfig& c) {
            std::stringstream ss;
            ss << "<ControllerConfig: "
               << c.control_frequency << "Hz control, "
               << c.policy_frequency << "Hz policy>";
            return ss.str();
        });

    // ========== Main Controller Class ==========

    py::class_<Lite3Controller>(m, "Lite3Controller", R"pbdoc(
        Main controller class for Lite3 quadruped robot.

        This class provides a high-level API for controlling the robot in both
        simulation and hardware modes. It handles state machine management,
        safety checks, threading, and data logging automatically.

        Args:
            use_sim (bool): If True, uses PyBullet simulation. If False, uses hardware.
            config (ControllerConfig): Optional configuration parameters.

        Example:
            >>> robot = Lite3Controller(use_sim=True)
            >>> robot.stand_up()
            >>> robot.set_velocity(0.5, 0.0, 0.0)
            >>> robot.run(ControlMode.POSITION_CONTROL)
    )pbdoc")
        // Constructors
        .def(py::init<bool, const ControllerConfig&>(),
             py::arg("use_sim") = true,
             py::arg("config") = ControllerConfig(),
             "Create controller with simulation/hardware mode")

        // Basic Control
        .def("initialize", &Lite3Controller::initialize,
             "Initialize and start robot interfaces")
        .def("stand_up", &Lite3Controller::standUp,
             py::arg("duration") = 2.0f,
             py::arg("blocking") = true,
             R"pbdoc(
                Stand up to default position with smooth interpolation.

                Args:
                    duration (float): Time to reach standing position [seconds]
                    blocking (bool): If True, waits until standing is complete
             )pbdoc")
        .def("set_velocity", &Lite3Controller::setVelocity,
             py::arg("vx"), py::arg("vy"), py::arg("vyaw"),
             R"pbdoc(
                Set target velocity command (normalized).

                Args:
                    vx (float): Forward velocity [-1.0, 1.0]
                    vy (float): Lateral velocity [-1.0, 1.0]
                    vyaw (float): Yaw rate [-1.0, 1.0]
             )pbdoc")
        .def("emergency_stop", &Lite3Controller::emergencyStop,
             "Emergency stop - immediately enable joint damping")
        .def("is_safe", &Lite3Controller::isSafe,
             "Check if robot is in safe state (orientation within limits)")
        .def("get_current_mode", &Lite3Controller::getCurrentMode,
             "Get current control mode")

        // State Reading
        .def("get_state", &Lite3Controller::getState,
             "Get complete robot state (IMU + joints + commands)")
        .def("get_joint_positions", &Lite3Controller::getJointPositions,
             "Get joint positions [rad] (12-DOF)")
        .def("get_joint_velocities", &Lite3Controller::getJointVelocities,
             "Get joint velocities [rad/s] (12-DOF)")
        .def("get_joint_torques", &Lite3Controller::getJointTorques,
             "Get joint torques [Nm] (12-DOF)")
        .def("get_orientation", &Lite3Controller::getOrientation,
             "Get base orientation (roll, pitch, yaw) [rad]")
        .def("get_angular_velocity", &Lite3Controller::getAngularVelocity,
             "Get base angular velocity [rad/s]")
        .def("get_acceleration", &Lite3Controller::getAcceleration,
             "Get base acceleration [m/s^2]")
        .def("get_command_velocities", &Lite3Controller::getCommandVelocities,
             "Get current command velocities (normalized)")
        .def("get_timestamp", &Lite3Controller::getTimestamp,
             "Get timestamp of current state [seconds]")

        // Advanced Control
        .def("set_joint_command", &Lite3Controller::setJointCommand,
             py::arg("target_pos"),
             py::arg("kp"),
             py::arg("kd"),
             py::arg("target_vel") = Eigen::VectorXf::Zero(12),
             py::arg("tau_ff") = Eigen::VectorXf::Zero(12),
             R"pbdoc(
                Send raw joint commands (PD control with feedforward).

                Args:
                    target_pos: Target joint positions [rad] (12-DOF)
                    kp: Proportional gains (12-DOF)
                    kd: Derivative gains (12-DOF)
                    target_vel: Target joint velocities [rad/s] (default: zero)
                    tau_ff: Feedforward torques [Nm] (default: zero)
             )pbdoc")
        .def("set_custom_policy", &Lite3Controller::setCustomPolicy,
             py::arg("callback"),
             py::arg("frequency") = 83.0f,
             R"pbdoc(
                Set a custom control policy via callback function.

                The callback will be called at the specified frequency with the
                current robot state, and should return the desired action.

                Args:
                    callback: Function(RobotBasicState) -> RobotAction
                    frequency: Control frequency [Hz]

                Example:
                    >>> def my_policy(state):
                    ...     action = pylite3.RobotAction()
                    ...     action.goal_joint_pos = np.zeros(12)
                    ...     action.kp = np.ones(12) * 30.0
                    ...     action.kd = np.ones(12) * 1.0
                    ...     return action
                    >>> robot.set_custom_policy(my_policy, 100.0)
             )pbdoc")
        .def("load_onnx_policy", &Lite3Controller::loadONNXPolicy,
             py::arg("model_path"),
             py::arg("obs_dim") = 45,
             py::arg("act_dim") = 12,
             "Load and configure an ONNX policy for RL control")

        // Control Loop
        .def("run", &Lite3Controller::run,
             py::arg("mode") = ControlMode::RL_CONTROL,
             py::call_guard<py::gil_scoped_release>(),  // Release GIL
             R"pbdoc(
                Start the control loop (blocking).

                This function will run the control loop in the current thread
                and block until stop() is called from another thread.

                Args:
                    mode: Control mode to use
             )pbdoc")
        .def("run_async", &Lite3Controller::runAsync,
             py::arg("mode") = ControlMode::RL_CONTROL,
             R"pbdoc(
                Start the control loop in background thread (non-blocking).

                This allows the main thread to continue and perform other tasks
                while the control loop runs in the background.

                Args:
                    mode: Control mode to use
             )pbdoc")
        .def("stop", &Lite3Controller::stop,
             "Stop the control loop")
        .def("is_running", &Lite3Controller::isRunning,
             "Check if control loop is currently running")

        // Configuration
        .def("set_default_pose", &Lite3Controller::setDefaultPose,
             py::arg("default_pos"),
             "Set default joint positions for standing pose")
        .def("set_default_gains", &Lite3Controller::setDefaultGains,
             py::arg("kp"), py::arg("kd"),
             "Set default PD gains")
        .def("set_safety_limits", &Lite3Controller::setSafetyLimits,
             py::arg("max_roll_deg"), py::arg("max_pitch_deg"),
             "Set safety limits (roll/pitch angles in degrees)")
        .def("set_logging", &Lite3Controller::setLogging,
             py::arg("enable"), py::arg("log_file") = "",
             "Enable or disable data logging")
        .def("set_control_frequency", &Lite3Controller::setControlFrequency,
             py::arg("frequency"),
             "Set control loop frequency [Hz]")
        .def("set_policy_frequency", &Lite3Controller::setPolicyFrequency,
             py::arg("frequency"),
             "Set policy execution frequency [Hz]")
        .def("get_config", &Lite3Controller::getConfig,
             py::return_value_policy::reference_internal,
             "Get current configuration")

        // Utilities
        .def("wait_for_pose", &Lite3Controller::waitForPose,
             py::arg("target_pos"),
             py::arg("tolerance") = 0.05f,
             py::arg("timeout") = 5.0f,
             py::call_guard<py::gil_scoped_release>(),
             R"pbdoc(
                Wait for robot to reach target pose (within tolerance).

                Args:
                    target_pos: Target joint positions
                    tolerance: Position tolerance [rad]
                    timeout: Maximum wait time [s]

                Returns:
                    True if reached, False if timeout
             )pbdoc")
        .def("get_performance_stats", &Lite3Controller::getPerformanceStats,
             "Get statistics about control loop performance")
        .def("reset", &Lite3Controller::reset,
             "Reset the controller to initial state")

        // Python-specific methods
        .def("__repr__", [](const Lite3Controller& c) {
            return "<Lite3Controller: " +
                   std::string(c.isRunning() ? "running" : "stopped") + ">";
        })
        .def("__enter__", [](Lite3Controller& c) -> Lite3Controller& {
            c.initialize();
            return c;
        })
        .def("__exit__", [](Lite3Controller& c, py::object, py::object, py::object) {
            c.stop();
        });

    // ========== Module-level Functions ==========

    m.def("version", []() {
        return "1.0.0";
    }, "Get PyLite3 version");

    m.def("create_default_config", []() {
        return ControllerConfig();
    }, "Create a default controller configuration");

    // Version info
    m.attr("__version__") = "1.0.0";
}
