"""
Example 6: Performance Benchmark
==================================
This example benchmarks the performance of different control modes.
"""
import pylite3
import numpy as np
import time


def benchmark_control_loop(robot, duration=10.0):
    """Benchmark control loop performance."""
    print(f"Running for {duration} seconds...")

    start_time = time.time()
    iterations = 0

    while time.time() - start_time < duration:
        state = robot.get_state()
        iterations += 1

    elapsed = time.time() - start_time
    frequency = iterations / elapsed

    print(f"  Iterations: {iterations}")
    print(f"  Elapsed: {elapsed:.2f} s")
    print(f"  Frequency: {frequency:.1f} Hz")

    return frequency


def benchmark_python_callback(duration=5.0):
    """Benchmark Python callback overhead."""
    print("\n=== Benchmark: Python Callback ===")

    call_count = [0]
    start_time = [0.0]
    times = []

    def benchmark_policy(state):
        call_count[0] += 1

        # Measure time
        if start_time[0] > 0:
            times.append(time.time() - start_time[0])
        start_time[0] = time.time()

        # Simple computation
        action = pylite3.RobotAction()
        action.goal_joint_pos = np.zeros(12, dtype=np.float32)
        action.kp = np.ones(12, dtype=np.float32) * 30.0
        action.kd = np.ones(12, dtype=np.float32) * 1.0
        return action

    robot = pylite3.Lite3Controller(use_sim=True)
    robot.initialize()
    robot.stand_up()

    robot.set_custom_policy(benchmark_policy, frequency=100.0)
    robot.run_async(pylite3.ControlMode.CUSTOM_CALLBACK)

    time.sleep(duration)

    robot.stop()

    print(f"Callback called {call_count[0]} times in {duration}s")
    print(f"Average frequency: {call_count[0] / duration:.1f} Hz")

    if times:
        times = np.array(times) * 1000  # Convert to ms
        print(f"Average callback interval: {np.mean(times):.2f} ms")
        print(f"Std dev: {np.std(times):.2f} ms")
        print(f"Min: {np.min(times):.2f} ms")
        print(f"Max: {np.max(times):.2f} ms")


def benchmark_state_reading(robot, num_reads=1000):
    """Benchmark state reading performance."""
    print("\n=== Benchmark: State Reading ===")

    start = time.time()
    for _ in range(num_reads):
        state = robot.get_state()
    elapsed = time.time() - start

    print(f"Read {num_reads} states in {elapsed:.3f} s")
    print(f"Average read time: {(elapsed / num_reads) * 1000:.3f} ms")
    print(f"Read frequency: {num_reads / elapsed:.1f} Hz")


def benchmark_command_sending(robot, num_commands=1000):
    """Benchmark command sending performance."""
    print("\n=== Benchmark: Command Sending ===")

    target_pos = np.zeros(12, dtype=np.float32)
    kp = np.ones(12, dtype=np.float32) * 30.0
    kd = np.ones(12, dtype=np.float32) * 1.0

    start = time.time()
    for _ in range(num_commands):
        robot.set_joint_command(target_pos, kp, kd)
    elapsed = time.time() - start

    print(f"Sent {num_commands} commands in {elapsed:.3f} s")
    print(f"Average send time: {(elapsed / num_commands) * 1000:.3f} ms")
    print(f"Command frequency: {num_commands / elapsed:.1f} Hz")


def main():
    print("=== PyLite3 Performance Benchmark ===\n")

    # Create robot
    robot = pylite3.Lite3Controller(use_sim=True)
    robot.initialize()

    print("Standing up...")
    robot.stand_up()

    # Benchmark state reading
    benchmark_state_reading(robot, num_reads=1000)

    # Benchmark command sending
    benchmark_command_sending(robot, num_commands=1000)

    # Benchmark Python callback
    benchmark_python_callback(duration=5.0)

    print("\n" + robot.get_performance_stats())

    print("\n=== Benchmark Complete ===")


if __name__ == "__main__":
    main()
