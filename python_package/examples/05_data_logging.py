"""
Example 5: Data Logging and Analysis
======================================
This example demonstrates data logging and visualization.
"""
import pylite3
import numpy as np
import time
import matplotlib.pyplot as plt
import pandas as pd

def main():
    print("=== Example 5: Data Logging ===\n")

    # Create config with logging enabled
    config = pylite3.ControllerConfig()
    config.enable_logging = True
    config.log_file = "lite3_data.csv"

    # Create robot with logging
    robot = pylite3.Lite3Controller(use_sim=True, config=config)
    robot.initialize()

    # Stand up
    print("Standing up...")
    robot.stand_up()

    # Walk in a pattern
    print("Walking in a pattern (logging data)...")
    robot.run_async(pylite3.ControlMode.POSITION_CONTROL)

    # Forward
    robot.set_velocity(0.5, 0.0, 0.0)
    time.sleep(3.0)

    # Turn left
    robot.set_velocity(0.3, 0.0, 0.5)
    time.sleep(2.0)

    # Forward again
    robot.set_velocity(0.5, 0.0, 0.0)
    time.sleep(3.0)

    # Stop
    robot.set_velocity(0.0, 0.0, 0.0)
    time.sleep(1.0)

    robot.stop()

    print(f"\nData logged to: {config.log_file}")
    print("Analyzing data...\n")

    # Load and analyze data
    try:
        df = pd.read_csv(config.log_file)
        print(f"Collected {len(df)} data points")
        print(f"Duration: {df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]:.2f} seconds")

        # Plot some data
        plot_data(df)

    except Exception as e:
        print(f"Error reading log file: {e}")

    print("\nExample complete!")

def plot_data(df):
    """Plot logged data."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # Plot IMU data
    axes[0].plot(df['timestamp'], df['roll'], label='Roll')
    axes[0].plot(df['timestamp'], df['pitch'], label='Pitch')
    axes[0].plot(df['timestamp'], df['yaw'], label='Yaw')
    axes[0].set_ylabel('Angle [rad]')
    axes[0].set_title('Base Orientation')
    axes[0].legend()
    axes[0].grid(True)

    # Plot joint positions (first 3 joints)
    for i in range(3):
        axes[1].plot(df['timestamp'], df[f'q{i}'], label=f'Joint {i}')
    axes[1].set_ylabel('Position [rad]')
    axes[1].set_title('Joint Positions (FL leg)')
    axes[1].legend()
    axes[1].grid(True)

    # Plot joint velocities (first 3 joints)
    for i in range(3):
        axes[2].plot(df['timestamp'], df[f'dq{i}'], label=f'Joint {i}')
    axes[2].set_xlabel('Time [s]')
    axes[2].set_ylabel('Velocity [rad/s]')
    axes[2].set_title('Joint Velocities (FL leg)')
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig('lite3_data_analysis.png', dpi=150)
    print("Plot saved to: lite3_data_analysis.png")
    plt.show()

if __name__ == "__main__":
    main()
