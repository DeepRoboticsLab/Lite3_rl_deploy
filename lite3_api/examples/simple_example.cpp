/**
 * @file simple_example.cpp
 * @brief Simple example of using Lite3Controller
 * @author HaiHa
 * @version 1.0
 * @date 2025-10-23
 */

#include "lite3_controller.hpp"
#include <iostream>
#include <chrono>
#include <thread>

using namespace lite3_api;

int main() {
    std::cout << "=== Lite3 Simple Example ===" << std::endl;

    // Create controller in simulation mode
    Lite3Controller robot(true);

    // Initialize
    robot.initialize();

    // Stand up
    std::cout << "Standing up..." << std::endl;
    robot.standUp(2.0f, true);

    // Wait a bit
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Walk forward slowly
    std::cout << "Walking forward..." << std::endl;
    robot.setVelocity(0.3f, 0.0f, 0.0f);

    // Run position control for 5 seconds
    robot.runAsync(ControlMode::POSITION_CONTROL);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Stop
    robot.stop();

    // Print performance stats
    std::cout << robot.getPerformanceStats() << std::endl;

    std::cout << "Example complete!" << std::endl;

    return 0;
}
