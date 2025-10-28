"""
Example 2: Custom Policy
=========================
This example shows how to implement a custom control policy in Python.
"""
import pylite3
import numpy as np
import time

def my_custom_policy(state):
    """
    Example custom policy: Simple PD control with sine wave modulation.

    Args:
        state: RobotBasicState containing all sensor data

    Returns:
        RobotAction with desired joint commands
    """
    # Default standing pose
    default_pose = np.array([0.0, -0.8, 1.6] * 4, dtype=np.float32)

    # Add a small sine wave to make joints oscillate
    t = time.time()
    modulation = 0.1 * np.sin(2 * np.pi * 0.5 * t)  # 0.5 Hz oscillation
    target_pos = default_pose + modulation

    # Create action
    action = pylite3.RobotAction()
    action.goal_joint_pos = target_pos
    action.goal_joint_vel = np.zeros(12, dtype=np.float32)
    action.kp = np.ones(12, dtype=np.float32) * 30.0
    action.kd = np.ones(12, dtype=np.float32) * 1.0
    action.tau_ff = np.zeros(12, dtype=np.float32)

    return action

def main():
    print("=== Example 2: Custom Policy ===\n")

    # Create robot controller
    robot = pylite3.Lite3Controller(use_sim=True)
    robot.initialize()

    # Stand up
    print("Standing up...")
    robot.stand_up()

    # Set custom policy
    print("Setting custom policy (sine wave oscillation)...")
    robot.set_custom_policy(my_custom_policy, frequency=100.0)

    # Run for 10 seconds
    print("Running custom policy for 10 seconds...")
    robot.run_async(pylite3.ControlMode.CUSTOM_CALLBACK)
    time.sleep(10.0)

    # Stop
    robot.stop()
    print("Example complete!")

if __name__ == "__main__":
    main()
