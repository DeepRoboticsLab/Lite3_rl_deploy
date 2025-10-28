/**
 * @file lite3_controller.cpp
 * @brief Implementation of Lite3Controller wrapper class
 * @author HaiHa
 * @version 1.0
 * @date 2025-10-23
 */

#include "lite3_controller.hpp"
#include "simulation/simulation_interface.hpp"
#include "hardware/hardware_interface.hpp"
#include "keyboard_interface.hpp"
#include "lite3_test_policy_runner_onnx.hpp"
#include "policy_runner_base.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <chrono>

namespace lite3_api {

using namespace std::chrono;

/**
 * @brief Private implementation (Pimpl idiom for ABI stability)
 */
struct Lite3Controller::Impl {
    // Configuration
    ControllerConfig config;

    // Interfaces
    std::shared_ptr<RobotInterface> robot_interface;
    std::shared_ptr<UserCommandInterface> user_interface;

    // State
    RobotBasicState current_state;
    UserCommand current_command;
    ControlMode current_mode;

    // Control
    PolicyCallback custom_policy;
    std::shared_ptr<PolicyRunnerBase> onnx_policy;

    // Threading
    std::atomic<bool> running{false};
    std::atomic<bool> initialized{false};
    std::thread control_thread;

    // Timing
    double timestamp = 0.0;
    int control_counter = 0;
    int policy_decimation = 12;  // Run policy every N control steps

    // Performance tracking
    std::vector<double> loop_times;
    double max_loop_time = 0.0;
    double total_loop_time = 0.0;
    int loop_count = 0;

    // Logging
    std::ofstream log_stream;
    bool logging_initialized = false;

    // Safety
    bool safety_triggered = false;

    // Stand up control
    bool standing_up = false;
    VecXf stand_start_pos;
    VecXf stand_target_pos;
    float stand_duration = 2.0f;
    float stand_elapsed = 0.0f;

    Impl() : current_mode(ControlMode::IDLE) {
        memset(&current_state, 0, sizeof(RobotBasicState));
        memset(&current_command, 0, sizeof(UserCommand));

        // Initialize vectors
        current_state.joint_pos = VecXf::Zero(12);
        current_state.joint_vel = VecXf::Zero(12);
        current_state.joint_tau = VecXf::Zero(12);
    }

    ~Impl() {
        if (log_stream.is_open()) {
            log_stream.close();
        }
    }
};

// ========== Constructor / Destructor ==========

Lite3Controller::Lite3Controller(bool use_sim, const ControllerConfig& config)
    : pimpl_(std::make_unique<Impl>()) {

    pimpl_->config = config;

    // Create appropriate robot interface
    if (use_sim) {
        #ifdef BUILD_SIMULATION
        pimpl_->robot_interface = std::make_shared<SimulationInterface>("Lite3");
        std::cout << "[Lite3Controller] Using PyBullet simulation" << std::endl;
        #else
        throw std::runtime_error("Simulation support not compiled. Build with -DBUILD_SIM=ON");
        #endif
    } else {
        pimpl_->robot_interface = std::make_shared<HardwareInterface>("Lite3");
        std::cout << "[Lite3Controller] Using hardware interface" << std::endl;
    }

    // Create default user interface (keyboard)
    pimpl_->user_interface = std::make_shared<KeyboardInterface>();

    // Calculate policy decimation
    pimpl_->policy_decimation = static_cast<int>(
        pimpl_->config.control_frequency / pimpl_->config.policy_frequency
    );

    std::cout << "[Lite3Controller] Initialized with:" << std::endl;
    std::cout << "  Control frequency: " << pimpl_->config.control_frequency << " Hz" << std::endl;
    std::cout << "  Policy frequency: " << pimpl_->config.policy_frequency << " Hz" << std::endl;
    std::cout << "  Policy decimation: " << pimpl_->policy_decimation << std::endl;
}

Lite3Controller::Lite3Controller(
    std::shared_ptr<RobotInterface> robot_interface,
    std::shared_ptr<UserCommandInterface> user_interface,
    const ControllerConfig& config)
    : pimpl_(std::make_unique<Impl>()) {

    pimpl_->config = config;
    pimpl_->robot_interface = robot_interface;
    pimpl_->user_interface = user_interface;

    if (!pimpl_->user_interface) {
        pimpl_->user_interface = std::make_shared<KeyboardInterface>();
    }

    pimpl_->policy_decimation = static_cast<int>(
        pimpl_->config.control_frequency / pimpl_->config.policy_frequency
    );

    std::cout << "[Lite3Controller] Initialized with custom interfaces" << std::endl;
}

Lite3Controller::~Lite3Controller() {
    stop();
}

// ========== Initialization ==========

void Lite3Controller::initialize() {
    if (pimpl_->initialized) {
        std::cout << "[Lite3Controller] Already initialized" << std::endl;
        return;
    }

    std::cout << "[Lite3Controller] Starting interfaces..." << std::endl;

    // Start robot interface
    pimpl_->robot_interface->Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Start user interface
    if (pimpl_->user_interface) {
        pimpl_->user_interface->Start();
    }

    // Wait for first state update
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Initialize logging if enabled
    if (pimpl_->config.enable_logging && !pimpl_->logging_initialized) {
        pimpl_->log_stream.open(pimpl_->config.log_file);
        if (pimpl_->log_stream.is_open()) {
            // Write CSV header
            pimpl_->log_stream << "timestamp,";
            pimpl_->log_stream << "roll,pitch,yaw,";
            pimpl_->log_stream << "wx,wy,wz,";
            pimpl_->log_stream << "ax,ay,az,";
            for (int i = 0; i < 12; ++i) {
                pimpl_->log_stream << "q" << i << ",";
            }
            for (int i = 0; i < 12; ++i) {
                pimpl_->log_stream << "dq" << i << ",";
            }
            for (int i = 0; i < 12; ++i) {
                pimpl_->log_stream << "tau" << i;
                if (i < 11) pimpl_->log_stream << ",";
            }
            pimpl_->log_stream << "\n";
            pimpl_->logging_initialized = true;
            std::cout << "[Lite3Controller] Logging enabled to: " << pimpl_->config.log_file << std::endl;
        }
    }

    pimpl_->initialized = true;
    std::cout << "[Lite3Controller] Initialization complete" << std::endl;
}

// ========== Basic Control ==========

void Lite3Controller::standUp(float duration, bool blocking) {
    if (!pimpl_->initialized) {
        initialize();
    }

    std::cout << "[Lite3Controller] Standing up..." << std::endl;

    // Get current joint positions
    pimpl_->stand_start_pos = pimpl_->robot_interface->GetJointPosition();
    pimpl_->stand_target_pos = pimpl_->config.default_pose;
    pimpl_->stand_duration = duration;
    pimpl_->stand_elapsed = 0.0f;
    pimpl_->standing_up = true;

    if (blocking) {
        // Blocking stand up
        auto start_time = steady_clock::now();
        float dt = 1.0f / pimpl_->config.control_frequency;

        while (pimpl_->stand_elapsed < duration) {
            updateState();

            // Interpolate position
            float t = pimpl_->stand_elapsed / duration;
            VecXf target = interpolatePose(pimpl_->stand_start_pos, pimpl_->stand_target_pos, t);

            // Send command
            MatXf cmd(12, 5);
            cmd.col(0) = pimpl_->config.default_kp;
            cmd.col(1) = target;
            cmd.col(2) = pimpl_->config.default_kd;
            cmd.col(3) = VecXf::Zero(12);
            cmd.col(4) = VecXf::Zero(12);
            pimpl_->robot_interface->SetJointCommand(cmd);

            pimpl_->stand_elapsed += dt;

            // Sleep to maintain frequency
            auto next_time = start_time + milliseconds(static_cast<int>(pimpl_->stand_elapsed * 1000));
            std::this_thread::sleep_until(next_time);
        }

        pimpl_->standing_up = false;
        std::cout << "[Lite3Controller] Stand up complete" << std::endl;
    }
}

void Lite3Controller::setVelocity(float vx, float vy, float vyaw) {
    // Clamp to [-1, 1]
    vx = std::max(-1.0f, std::min(1.0f, vx));
    vy = std::max(-1.0f, std::min(1.0f, vy));
    vyaw = std::max(-1.0f, std::min(1.0f, vyaw));

    pimpl_->current_command.forward_vel_scale = vx;
    pimpl_->current_command.side_vel_scale = vy;
    pimpl_->current_command.turnning_vel_scale = vyaw;

    pimpl_->current_state.cmd_vel_normlized = Vec3f(vx, vy, vyaw);
}

void Lite3Controller::emergencyStop() {
    std::cout << "[Lite3Controller] EMERGENCY STOP!" << std::endl;
    pimpl_->safety_triggered = true;
    stop();

    // Set all joints to damping mode
    MatXf cmd(12, 5);
    cmd.col(0) = VecXf::Zero(12);  // kp = 0
    cmd.col(1) = VecXf::Zero(12);
    cmd.col(2) = VecXf::Constant(12, 5.0f);  // kd = 5
    cmd.col(3) = VecXf::Zero(12);
    cmd.col(4) = VecXf::Zero(12);
    pimpl_->robot_interface->SetJointCommand(cmd);
}

bool Lite3Controller::isSafe() const {
    return checkSafety();
}

ControlMode Lite3Controller::getCurrentMode() const {
    return pimpl_->current_mode;
}

// ========== State Reading ==========

RobotBasicState Lite3Controller::getState() const {
    return pimpl_->current_state;
}

VecXf Lite3Controller::getJointPositions() const {
    return pimpl_->robot_interface->GetJointPosition();
}

VecXf Lite3Controller::getJointVelocities() const {
    return pimpl_->robot_interface->GetJointVelocity();
}

VecXf Lite3Controller::getJointTorques() const {
    return pimpl_->robot_interface->GetJointTorque();
}

Vec3f Lite3Controller::getOrientation() const {
    return pimpl_->robot_interface->GetImuRpy();
}

Vec3f Lite3Controller::getAngularVelocity() const {
    return pimpl_->robot_interface->GetImuOmega();
}

Vec3f Lite3Controller::getAcceleration() const {
    return pimpl_->robot_interface->GetImuAcc();
}

Vec3f Lite3Controller::getCommandVelocities() const {
    return pimpl_->current_state.cmd_vel_normlized;
}

double Lite3Controller::getTimestamp() const {
    return pimpl_->robot_interface->GetInterfaceTimeStamp();
}

// ========== Advanced Control ==========

void Lite3Controller::setJointCommand(
    const VecXf& target_pos,
    const VecXf& kp,
    const VecXf& kd,
    const VecXf& target_vel,
    const VecXf& tau_ff) {

    MatXf cmd(12, 5);
    cmd.col(0) = kp;
    cmd.col(1) = target_pos;
    cmd.col(2) = kd;
    cmd.col(3) = target_vel;
    cmd.col(4) = tau_ff;
    pimpl_->robot_interface->SetJointCommand(cmd);
}

void Lite3Controller::setCustomPolicy(PolicyCallback callback, float frequency) {
    pimpl_->custom_policy = callback;
    pimpl_->config.policy_frequency = frequency;
    pimpl_->policy_decimation = static_cast<int>(
        pimpl_->config.control_frequency / frequency
    );
    std::cout << "[Lite3Controller] Custom policy set at " << frequency << " Hz" << std::endl;
}

void Lite3Controller::loadONNXPolicy(const std::string& model_path, int obs_dim, int act_dim) {
    std::cout << "[Lite3Controller] Loading ONNX policy: " << model_path << std::endl;

    try {
        pimpl_->onnx_policy = std::make_shared<Lite3TestPolicyRunnerONNX>("loaded_policy");
        pimpl_->onnx_policy->DisplayPolicyInfo();
        std::cout << "[Lite3Controller] ONNX policy loaded successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[Lite3Controller] Failed to load ONNX policy: " << e.what() << std::endl;
        throw;
    }
}

// ========== Control Loop ==========

void Lite3Controller::run(ControlMode mode) {
    if (!pimpl_->initialized) {
        initialize();
    }

    pimpl_->current_mode = mode;
    pimpl_->running = true;

    std::cout << "[Lite3Controller] Starting control loop in mode: " << static_cast<int>(mode) << std::endl;

    controlLoop();
}

void Lite3Controller::runAsync(ControlMode mode) {
    if (!pimpl_->initialized) {
        initialize();
    }

    if (pimpl_->running) {
        std::cout << "[Lite3Controller] Already running" << std::endl;
        return;
    }

    pimpl_->current_mode = mode;
    pimpl_->running = true;

    pimpl_->control_thread = std::thread([this]() {
        controlLoop();
    });

    std::cout << "[Lite3Controller] Control loop started in background (mode: "
              << static_cast<int>(mode) << ")" << std::endl;
}

void Lite3Controller::stop() {
    if (!pimpl_->running) {
        return;
    }

    std::cout << "[Lite3Controller] Stopping control loop..." << std::endl;
    pimpl_->running = false;

    if (pimpl_->control_thread.joinable()) {
        pimpl_->control_thread.join();
    }

    std::cout << "[Lite3Controller] Control loop stopped" << std::endl;
}

bool Lite3Controller::isRunning() const {
    return pimpl_->running;
}

// ========== Configuration ==========

void Lite3Controller::setDefaultPose(const VecXf& default_pos) {
    if (default_pos.size() != 12) {
        throw std::invalid_argument("Default pose must have 12 elements");
    }
    pimpl_->config.default_pose = default_pos;
}

void Lite3Controller::setDefaultGains(const VecXf& kp, const VecXf& kd) {
    if (kp.size() != 12 || kd.size() != 12) {
        throw std::invalid_argument("Gains must have 12 elements");
    }
    pimpl_->config.default_kp = kp;
    pimpl_->config.default_kd = kd;
}

void Lite3Controller::setSafetyLimits(float max_roll_deg, float max_pitch_deg) {
    pimpl_->config.max_roll_deg = max_roll_deg;
    pimpl_->config.max_pitch_deg = max_pitch_deg;
    std::cout << "[Lite3Controller] Safety limits set: roll=" << max_roll_deg
              << "°, pitch=" << max_pitch_deg << "°" << std::endl;
}

void Lite3Controller::setLogging(bool enable, const std::string& log_file) {
    pimpl_->config.enable_logging = enable;
    if (!log_file.empty()) {
        pimpl_->config.log_file = log_file;
    }
}

void Lite3Controller::setControlFrequency(float frequency) {
    pimpl_->config.control_frequency = frequency;
    pimpl_->policy_decimation = static_cast<int>(frequency / pimpl_->config.policy_frequency);
}

void Lite3Controller::setPolicyFrequency(float frequency) {
    pimpl_->config.policy_frequency = frequency;
    pimpl_->policy_decimation = static_cast<int>(
        pimpl_->config.control_frequency / frequency
    );
}

const ControllerConfig& Lite3Controller::getConfig() const {
    return pimpl_->config;
}

// ========== Utilities ==========

bool Lite3Controller::waitForPose(const VecXf& target_pos, float tolerance, float timeout) {
    auto start = steady_clock::now();

    while (true) {
        VecXf current = getJointPositions();
        float error = (current - target_pos).norm();

        if (error < tolerance) {
            return true;
        }

        auto elapsed = duration_cast<milliseconds>(steady_clock::now() - start).count() / 1000.0f;
        if (elapsed > timeout) {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

std::string Lite3Controller::getPerformanceStats() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "Performance Statistics:\n";
    oss << "  Loop count: " << pimpl_->loop_count << "\n";

    if (pimpl_->loop_count > 0) {
        double avg_time = pimpl_->total_loop_time / pimpl_->loop_count;
        oss << "  Avg loop time: " << (avg_time * 1000.0) << " ms\n";
        oss << "  Max loop time: " << (pimpl_->max_loop_time * 1000.0) << " ms\n";
        oss << "  Avg frequency: " << (1.0 / avg_time) << " Hz\n";
    }

    return oss.str();
}

void Lite3Controller::reset() {
    stop();
    pimpl_->control_counter = 0;
    pimpl_->safety_triggered = false;
    pimpl_->loop_count = 0;
    pimpl_->total_loop_time = 0.0;
    pimpl_->max_loop_time = 0.0;
    std::cout << "[Lite3Controller] Reset complete" << std::endl;
}

// ========== Private Methods ==========

void Lite3Controller::controlLoop() {
    auto last_time = steady_clock::now();
    double dt = 1.0 / pimpl_->config.control_frequency;

    while (pimpl_->running) {
        auto loop_start = steady_clock::now();

        // Update state
        updateState();

        // Safety check
        if (!checkSafety()) {
            std::cerr << "[Lite3Controller] Safety check failed!" << std::endl;
            emergencyStop();
            break;
        }

        // Execute control
        executeControl();

        // Log data
        if (pimpl_->config.enable_logging) {
            logData();
        }

        pimpl_->control_counter++;
        pimpl_->loop_count++;

        // Performance tracking
        auto loop_end = steady_clock::now();
        double loop_time = duration_cast<microseconds>(loop_end - loop_start).count() / 1e6;
        pimpl_->total_loop_time += loop_time;
        pimpl_->max_loop_time = std::max(pimpl_->max_loop_time, loop_time);

        // Sleep to maintain frequency
        auto next_time = last_time + microseconds(static_cast<int>(dt * 1e6));
        std::this_thread::sleep_until(next_time);
        last_time = next_time;
    }
}

bool Lite3Controller::checkSafety() const {
    Vec3f rpy = pimpl_->robot_interface->GetImuRpy();
    float roll_deg = std::abs(rpy(0)) * 180.0f / M_PI;
    float pitch_deg = std::abs(rpy(1)) * 180.0f / M_PI;

    return (roll_deg < pimpl_->config.max_roll_deg) &&
           (pitch_deg < pimpl_->config.max_pitch_deg);
}

void Lite3Controller::updateState() {
    pimpl_->current_state.base_rpy = pimpl_->robot_interface->GetImuRpy();
    pimpl_->current_state.base_omega = pimpl_->robot_interface->GetImuOmega();
    pimpl_->current_state.base_acc = pimpl_->robot_interface->GetImuAcc();
    pimpl_->current_state.joint_pos = pimpl_->robot_interface->GetJointPosition();
    pimpl_->current_state.joint_vel = pimpl_->robot_interface->GetJointVelocity();
    pimpl_->current_state.joint_tau = pimpl_->robot_interface->GetJointTorque();

    // Compute rotation matrix and projected gravity
    float roll = pimpl_->current_state.base_rpy(0);
    float pitch = pimpl_->current_state.base_rpy(1);
    float yaw = pimpl_->current_state.base_rpy(2);

    Mat3f Rx, Ry, Rz;
    Rx << 1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);
    Ry << cos(pitch), 0, sin(pitch),
          0, 1, 0,
          -sin(pitch), 0, cos(pitch);
    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw), cos(yaw), 0,
          0, 0, 1;

    pimpl_->current_state.base_rot_mat = Rz * Ry * Rx;
    Vec3f gravity(0, 0, -1);
    pimpl_->current_state.projected_gravity = pimpl_->current_state.base_rot_mat.inverse() * gravity;

    pimpl_->timestamp = pimpl_->robot_interface->GetInterfaceTimeStamp();
}

void Lite3Controller::executeControl() {
    switch (pimpl_->current_mode) {
        case ControlMode::IDLE: {
            // Do nothing, joints unpowered
            MatXf cmd(12, 5);
            cmd.setZero();
            pimpl_->robot_interface->SetJointCommand(cmd);
            break;
        }

        case ControlMode::POSITION_CONTROL: {
            // Simple position control to default pose
            MatXf cmd(12, 5);
            cmd.col(0) = pimpl_->config.default_kp;
            cmd.col(1) = pimpl_->config.default_pose;
            cmd.col(2) = pimpl_->config.default_kd;
            cmd.col(3) = VecXf::Zero(12);
            cmd.col(4) = VecXf::Zero(12);
            pimpl_->robot_interface->SetJointCommand(cmd);
            break;
        }

        case ControlMode::RL_CONTROL: {
            // Run ONNX policy at specified frequency
            if (pimpl_->onnx_policy &&
                pimpl_->control_counter % pimpl_->policy_decimation == 0) {
                RobotAction action = pimpl_->onnx_policy->GetRobotAction(pimpl_->current_state);
                MatXf cmd = action.ConvertToMat();
                pimpl_->robot_interface->SetJointCommand(cmd);
            }
            break;
        }

        case ControlMode::CUSTOM_CALLBACK: {
            // Run custom policy at specified frequency
            if (pimpl_->custom_policy &&
                pimpl_->control_counter % pimpl_->policy_decimation == 0) {
                RobotAction action = pimpl_->custom_policy(pimpl_->current_state);
                MatXf cmd = action.ConvertToMat();
                pimpl_->robot_interface->SetJointCommand(cmd);
            }
            break;
        }
    }
}

void Lite3Controller::logData() {
    if (!pimpl_->log_stream.is_open()) {
        return;
    }

    pimpl_->log_stream << pimpl_->timestamp << ",";

    // IMU data
    Vec3f rpy = pimpl_->current_state.base_rpy;
    pimpl_->log_stream << rpy(0) << "," << rpy(1) << "," << rpy(2) << ",";

    Vec3f omega = pimpl_->current_state.base_omega;
    pimpl_->log_stream << omega(0) << "," << omega(1) << "," << omega(2) << ",";

    Vec3f acc = pimpl_->current_state.base_acc;
    pimpl_->log_stream << acc(0) << "," << acc(1) << "," << acc(2) << ",";

    // Joint data
    for (int i = 0; i < 12; ++i) {
        pimpl_->log_stream << pimpl_->current_state.joint_pos(i) << ",";
    }
    for (int i = 0; i < 12; ++i) {
        pimpl_->log_stream << pimpl_->current_state.joint_vel(i) << ",";
    }
    for (int i = 0; i < 12; ++i) {
        pimpl_->log_stream << pimpl_->current_state.joint_tau(i);
        if (i < 11) pimpl_->log_stream << ",";
    }
    pimpl_->log_stream << "\n";
}

VecXf Lite3Controller::interpolatePose(const VecXf& start, const VecXf& end, float t) {
    // Smooth interpolation using cubic easing
    t = std::max(0.0f, std::min(1.0f, t));
    float smooth_t = t * t * (3.0f - 2.0f * t);  // Smoothstep
    return start + smooth_t * (end - start);
}

} // namespace lite3_api
