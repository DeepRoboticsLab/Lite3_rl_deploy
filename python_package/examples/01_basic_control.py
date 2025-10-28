"""
Example 1: Basic Control
=========================
This example demonstrates basic control of the Lite3 robot using the Python API.
"""
import pylite3
import time

def main():
    print("=== Example 1: Basic Control ===\n")

    # Create controller in simulation mode
    robot = pylite3.Lite3Controller(use_sim=True)

    # Initialize (starts interfaces)
    robot.initialize()

    # Stand up
    print("Standing up...")
    robot.stand_up(duration=2.0, blocking=True)
    print("Standing complete!\n")

    # Wait a bit
    time.sleep(1.0)

    # Walk forward
    print("Walking forward for 5 seconds...")
    robot.set_velocity(vx=0.5, vy=0.0, vyaw=0.0)

    # Run position control
    robot.run_async(pylite3.ControlMode.POSITION_CONTROL)
    time.sleep(5.0)

    # Turn left
    print("Turning left for 3 seconds...")
    robot.set_velocity(vx=0.3, vy=0.0, vyaw=0.5)
    time.sleep(3.0)

    # Stop
    print("Stopping...")
    robot.set_velocity(vx=0.0, vy=0.0, vyaw=0.0)
    time.sleep(2.0)

    robot.stop()

    # Print performance stats
    print("\n" + robot.get_performance_stats())

    print("\nExample complete!")

if __name__ == "__main__":
    main()
