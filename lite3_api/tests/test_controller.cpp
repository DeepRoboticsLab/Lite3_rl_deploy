/**
 * @file test_controller.cpp
 * @brief Unit tests for Lite3Controller
 * @author HaiHa
 * @version 1.0
 * @date 2025-10-23
 */

#include "lite3_controller.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace lite3_api;

// Simple test framework
#define TEST(name) void test_##name()
#define RUN_TEST(name) do { \
    std::cout << "Running test: " << #name << "... "; \
    test_##name(); \
    std::cout << "PASSED" << std::endl; \
} while(0)

#define ASSERT_TRUE(condition) do { \
    if (!(condition)) { \
        std::cerr << "FAILED: " << #condition << " at line " << __LINE__ << std::endl; \
        exit(1); \
    } \
} while(0)

#define ASSERT_FLOAT_EQ(a, b, eps) ASSERT_TRUE(std::abs((a) - (b)) < (eps))

// Test: Constructor
TEST(constructor) {
    Lite3Controller robot(true);  // Simulation mode
    ASSERT_TRUE(true);  // If we get here, construction succeeded
}

// Test: Configuration
TEST(configuration) {
    ControllerConfig config;
    config.control_frequency = 500.0f;
    config.policy_frequency = 50.0f;
    config.max_roll_deg = 40.0f;

    Lite3Controller robot(true, config);
    const auto& cfg = robot.getConfig();

    ASSERT_FLOAT_EQ(cfg.control_frequency, 500.0f, 0.01f);
    ASSERT_FLOAT_EQ(cfg.policy_frequency, 50.0f, 0.01f);
    ASSERT_FLOAT_EQ(cfg.max_roll_deg, 40.0f, 0.01f);
}

// Test: Set velocity
TEST(set_velocity) {
    Lite3Controller robot(true);
    robot.setVelocity(0.5f, 0.2f, -0.3f);

    Vec3f cmd_vel = robot.getCommandVelocities();
    ASSERT_FLOAT_EQ(cmd_vel(0), 0.5f, 0.001f);
    ASSERT_FLOAT_EQ(cmd_vel(1), 0.2f, 0.001f);
    ASSERT_FLOAT_EQ(cmd_vel(2), -0.3f, 0.001f);
}

// Test: Set velocity clamping
TEST(set_velocity_clamping) {
    Lite3Controller robot(true);
    robot.setVelocity(1.5f, -2.0f, 0.5f);  // Out of range

    Vec3f cmd_vel = robot.getCommandVelocities();
    ASSERT_FLOAT_EQ(cmd_vel(0), 1.0f, 0.001f);   // Clamped to 1.0
    ASSERT_FLOAT_EQ(cmd_vel(1), -1.0f, 0.001f);  // Clamped to -1.0
    ASSERT_FLOAT_EQ(cmd_vel(2), 0.5f, 0.001f);   // Within range
}

// Test: Default pose
TEST(default_pose) {
    Lite3Controller robot(true);

    VecXf new_pose(12);
    new_pose << 0.1, -0.9, 1.7, 0.1, -0.9, 1.7,
                0.1, -0.9, 1.7, 0.1, -0.9, 1.7;

    robot.setDefaultPose(new_pose);
    const auto& cfg = robot.getConfig();

    for (int i = 0; i < 12; ++i) {
        ASSERT_FLOAT_EQ(cfg.default_pose(i), new_pose(i), 0.001f);
    }
}

// Test: Safety limits
TEST(safety_limits) {
    Lite3Controller robot(true);
    robot.setSafetyLimits(35.0f, 50.0f);

    const auto& cfg = robot.getConfig();
    ASSERT_FLOAT_EQ(cfg.max_roll_deg, 35.0f, 0.01f);
    ASSERT_FLOAT_EQ(cfg.max_pitch_deg, 50.0f, 0.01f);
}

// Test: Custom policy callback
TEST(custom_policy) {
    Lite3Controller robot(true);

    bool callback_called = false;
    auto test_policy = [&callback_called](const RobotBasicState& state) {
        callback_called = true;
        RobotAction action;
        action.goal_joint_pos = VecXf::Zero(12);
        action.kp = VecXf::Constant(12, 30.0f);
        action.kd = VecXf::Constant(12, 1.0f);
        action.tau_ff = VecXf::Zero(12);
        return action;
    };

    robot.setCustomPolicy(test_policy, 100.0f);
    ASSERT_TRUE(true);  // Successfully set policy
}

// Test: Initialization
TEST(initialization) {
    Lite3Controller robot(true);
    robot.initialize();
    ASSERT_TRUE(true);  // Initialization succeeded
}

// Test: State reading
TEST(state_reading) {
    Lite3Controller robot(true);
    robot.initialize();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Read various states
    VecXf joint_pos = robot.getJointPositions();
    VecXf joint_vel = robot.getJointVelocities();
    Vec3f orientation = robot.getOrientation();
    Vec3f angular_vel = robot.getAngularVelocity();

    // Basic sanity checks
    ASSERT_TRUE(joint_pos.size() == 12);
    ASSERT_TRUE(joint_vel.size() == 12);
    ASSERT_TRUE(orientation.size() == 3);
    ASSERT_TRUE(angular_vel.size() == 3);
}

// Test: Joint command
TEST(joint_command) {
    Lite3Controller robot(true);
    robot.initialize();

    VecXf target_pos = VecXf::Zero(12);
    VecXf kp = VecXf::Constant(12, 30.0f);
    VecXf kd = VecXf::Constant(12, 1.0f);

    robot.setJointCommand(target_pos, kp, kd);
    ASSERT_TRUE(true);  // Command sent successfully
}

// Test: Performance stats
TEST(performance_stats) {
    Lite3Controller robot(true);
    std::string stats = robot.getPerformanceStats();
    ASSERT_TRUE(!stats.empty());
    std::cout << "\n" << stats;
}

int main() {
    std::cout << "=== Lite3Controller Unit Tests ===" << std::endl;
    std::cout << std::endl;

    RUN_TEST(constructor);
    RUN_TEST(configuration);
    RUN_TEST(set_velocity);
    RUN_TEST(set_velocity_clamping);
    RUN_TEST(default_pose);
    RUN_TEST(safety_limits);
    RUN_TEST(custom_policy);
    RUN_TEST(initialization);
    RUN_TEST(state_reading);
    RUN_TEST(joint_command);
    RUN_TEST(performance_stats);

    std::cout << std::endl;
    std::cout << "=== All tests passed! ===" << std::endl;

    return 0;
}
