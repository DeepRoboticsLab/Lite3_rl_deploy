# PyLite3 - Python API for Lite3 Quadruped Robot

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://github.com/DeepRoboticsLab/Lite3_rl_deploy)
[![Python](https://img.shields.io/badge/python-3.8+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-MIT-orange.svg)](LICENSE)

PyLite3 is a high-level Python API for controlling the DeepRobotics Lite3 quadruped robot in both simulation and hardware modes. It provides a simple, intuitive interface that abstracts away the complexity of low-level control, making it easy to develop and test new algorithms.

## Features

- **Simple API**: Easy-to-use Python interface for robot control
- **Simulation & Hardware**: Seamlessly switch between PyBullet simulation and real hardware
- **Custom Policies**: Implement control policies in pure Python
- **PyTorch Integration**: Use PyTorch models directly without ONNX conversion
- **Gymnasium Environment**: Standard RL environment for training with any RL library
- **Data Logging**: Built-in logging and visualization tools
- **Real-time Control**: Low-latency control loop with configurable frequencies
- **Safety Features**: Automatic safety checks and emergency stop

## Installation

### Prerequisites

```bash
# System dependencies (Ubuntu)
sudo apt-get install libdw-dev

# Python dependencies
pip install numpy matplotlib pybullet
```

### Build from Source

```bash
# Clone repository
git clone --recurse-submodule https://github.com/DeepRoboticsLab/Lite3_rl_deploy.git
cd Lite3_rl_deploy

# Install pybind11
pip install pybind11

# Build C++ library and Python bindings
mkdir build && cd build
cmake .. -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON -DBUILD_PYTHON_BINDINGS=ON
make -j

# Install Python package
cd ../python_package
pip install -e .
```

### Verify Installation

```python
import pylite3
print(pylite3.version())
```

## Quick Start

### Basic Control

```python
import pylite3

# Create controller (simulation mode)
robot = pylite3.Lite3Controller(use_sim=True)

# Initialize
robot.initialize()

# Stand up
robot.stand_up()

# Walk forward
robot.set_velocity(vx=0.5, vy=0.0, vyaw=0.0)
robot.run(pylite3.ControlMode.POSITION_CONTROL)
```

### Custom Python Policy

```python
import pylite3
import numpy as np

def my_policy(state):
    """Custom control policy."""
    action = pylite3.RobotAction()
    action.goal_joint_pos = np.zeros(12)  # Your algorithm here
    action.kp = np.ones(12) * 30.0
    action.kd = np.ones(12) * 1.0
    return action

robot = pylite3.Lite3Controller(use_sim=True)
robot.initialize()
robot.stand_up()
robot.set_custom_policy(my_policy, frequency=100.0)
robot.run(pylite3.ControlMode.CUSTOM_CALLBACK)
```

### PyTorch Policy (No ONNX!)

```python
import pylite3
import torch
import numpy as np

class MyTorchPolicy:
    def __init__(self, model_path):
        self.model = torch.jit.load(model_path)
        self.model.eval()

    def __call__(self, state):
        # Prepare observation
        obs = np.concatenate([state.base_omega, state.joint_pos, state.joint_vel])

        # Run PyTorch inference
        with torch.no_grad():
            action = self.model(torch.from_numpy(obs).float()).numpy()

        # Return RobotAction
        robot_action = pylite3.RobotAction()
        robot_action.goal_joint_pos = action
        robot_action.kp = np.ones(12) * 30.0
        robot_action.kd = np.ones(12) * 1.0
        return robot_action

policy = MyTorchPolicy("policy.pt")
robot = pylite3.Lite3Controller(use_sim=True)
robot.initialize()
robot.stand_up()
robot.set_custom_policy(policy)
robot.run(pylite3.ControlMode.CUSTOM_CALLBACK)
```

### Gymnasium Environment

```python
import pylite3

# Create Gym environment
env = pylite3.Lite3GymEnv(use_sim=True)

# Standard Gym interface
obs, info = env.reset()
for _ in range(1000):
    action = env.action_space.sample()  # Your policy here
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        obs, info = env.reset()
```

### Training with Stable-Baselines3

```python
from stable_baselines3 import PPO
import pylite3

env = pylite3.Lite3GymEnv(use_sim=True)
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100000)
model.save("lite3_ppo")
```

## Examples

See the `examples/` directory for more comprehensive examples:

- `01_basic_control.py` - Basic robot control
- `02_custom_policy.py` - Custom policy implementation
- `03_pytorch_policy.py` - Using PyTorch models
- `04_gym_training.py` - Gymnasium environment and RL training
- `05_data_logging.py` - Data logging and visualization

## API Documentation

### `Lite3Controller`

Main controller class for robot control.

**Constructor:**
```python
Lite3Controller(use_sim=True, config=ControllerConfig())
```

**Key Methods:**
- `initialize()` - Initialize robot interfaces
- `stand_up(duration=2.0)` - Stand up to default pose
- `set_velocity(vx, vy, vyaw)` - Set velocity command
- `set_custom_policy(callback, frequency)` - Set custom policy
- `run(mode)` - Run control loop (blocking)
- `run_async(mode)` - Run control loop (non-blocking)
- `stop()` - Stop control loop
- `get_state()` - Get current robot state
- `get_joint_positions()` - Get joint positions
- `emergency_stop()` - Emergency stop

### `Lite3GymEnv`

Gymnasium environment for RL training.

**Constructor:**
```python
Lite3GymEnv(use_sim=True, max_episode_steps=1000, control_frequency=50.0)
```

**Spaces:**
- Observation: `Box(45,)` - IMU, joints, commands
- Action: `Box(12,)` - Joint position targets (normalized)

## Configuration

```python
config = pylite3.ControllerConfig()
config.control_frequency = 1000.0  # Hz
config.policy_frequency = 83.0     # Hz
config.max_roll_deg = 30.0         # degrees
config.max_pitch_deg = 45.0        # degrees
config.enable_logging = True
config.log_file = "robot_data.csv"

robot = pylite3.Lite3Controller(use_sim=True, config=config)
```

## Hardware Deployment

```python
# Use hardware mode (requires SSH connection to robot)
robot = pylite3.Lite3Controller(use_sim=False)
robot.initialize()
robot.stand_up()
# ... your control code
```

## Data Logging

```python
config = pylite3.ControllerConfig()
config.enable_logging = True
config.log_file = "data.csv"

robot = pylite3.Lite3Controller(use_sim=True, config=config)
# ... run robot
# Data saved to data.csv (timestamp, IMU, joints, etc.)
```

## Performance

- Control loop: ~1000 Hz
- Policy frequency: Configurable (default 83 Hz)
- Python callback overhead: ~1-2 ms
- Suitable for real-time control applications

## Troubleshooting

### Import Error

If you get an import error:
```
ImportError: cannot import name 'pylite3'
```

Make sure you built the C++ library with Python bindings enabled:
```bash
cmake .. -DBUILD_PYTHON_BINDINGS=ON -DBUILD_SIM=ON
make -j
```

### Simulation Not Starting

Ensure PyBullet simulation is running in a separate terminal:
```bash
cd interface/robot/simulation
python3 pybullet_simulation.py
```

## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## License

MIT License. See [LICENSE](LICENSE) for details.

## Citation

```bibtex
@software{pylite3,
  title={PyLite3: Python API for Lite3 Quadruped Robot},
  author={DeepRobotics},
  year={2024},
  url={https://github.com/DeepRoboticsLab/Lite3_rl_deploy}
}
```

## Support

- Documentation: [docs/](docs/)
- Issues: [GitHub Issues](https://github.com/DeepRoboticsLab/Lite3_rl_deploy/issues)
- Discord: [Join our community](https://discord.gg/gdM9mQutC8)

## Acknowledgments

Built on top of the excellent work by DeepRobotics Lab.
