# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Lite3 RL Deploy is a reinforcement learning policy deployment framework for the Lite3 quadruped robot. It supports both simulation (PyBullet, MuJoCo) and real hardware execution.

## Build Commands

### Simulation (x86/Ubuntu)
```bash
# Install prerequisites (once)
sudo apt-get install libdw-dev
wget https://raw.githubusercontent.com/bombela/backward-cpp/master/backward.hpp
sudo mv backward.hpp /usr/include
pip install pybullet "numpy < 2.0" mujoco

# Build
mkdir build && cd build
cmake .. -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON -DSEND_REMOTE=OFF
make -j
```

### Real Hardware (ARM, run on robot)
```bash
mkdir build && cd build
cmake .. -DBUILD_PLATFORM=arm -DBUILD_SIM=OFF -DSEND_REMOTE=OFF
make -j
```

### CMake Options
- `-DBUILD_PLATFORM`: `x86` (Ubuntu dev) or `arm` (robot hardware)
- `-DBUILD_SIM`: `ON` for simulation, `OFF` for real hardware
- `-DUSE_PYBULLET=ON` / `-DUSE_MJCPP=ON` / `-DUSE_RAISIM=ON`: select simulator
- `-DSEND_REMOTE=ON`: auto-SCP binary to robot after build

## Running

**Simulation requires two terminals:**
```bash
# Terminal 1 — pick one simulator:
cd interface/robot/simulation && python pybullet_simulation.py
# or:
cd interface/robot/simulation && python mujoco_simulation.py

# Terminal 2 — control loop:
cd build && ./rl_deploy
```

**Keyboard controls (simulation):** `z` = stand up, `c` = RL control, `wasd` = movement, `qe` = rotation

**Real hardware:** Connect to robot WiFi (`Lite*******`, password `12345678`), deploy via `scp -r ~/Lite3_rl_deploy ysc@192.168.2.1:~/`, then SSH and run.

## Model Conversion (PyTorch → ONNX)
```bash
pip install torch numpy onnx onnxruntime
cd policy/
python pt2onnx.py   # outputs policy.onnx in same folder
```

## Architecture

### State Machine (`state_machine/`)
The central control loop. States transition linearly: **Idle → StandUp → RLControl → JointDamping → Idle**

- `state_machine.hpp`: Orchestrates transitions, user input, data streaming
- `state_base.h`: Abstract base with `Run()`, `OnEnter()`, `OnExit()`, `GetNextStateName()`
- `rl_control_state_onnx.hpp`: Runs ONNX policy in a separate thread; monitors roll/pitch safety limits (±30°/±45°) and falls back to JointDamping on violation

### Interface Layer (`interface/`)
Abstracts both robot platforms and user input:
- **Robot:** `robot_interface.h` (abstract) → `HardwareInterface` (real robot via Lite3 Motion SDK) or `SimulationInterface` (PyBullet via UDP) or `MujocoInterface` (MuJoCo C++)
- **User input:** `KeyboardInterface` (simulation default), `RetroidGamepadInterface` / `SkydroidGamepadInterface` (hardware default)
- To switch hardware to keyboard: change `state_machine.hpp:121` to `uc_ptr_ = std::make_shared<KeyboardInterface>();`

### Policy Runner (`run_policy/`)
- `policy_runner_base.hpp`: Abstract interface (`GetRobotAction()`, decimation support)
- `lite3_test_policy_runner_onnx.h` + `.cpp`: Loads `policy/ppo/policy.onnx` via onnxruntime; processes 45-dim observations, outputs joint torques + PD gains. All Ort types are hidden behind a PIMPL struct in the `.cpp` so onnxruntime headers are only parsed by that one translation unit.

### Simulation Communication
PyBullet/MuJoCo simulators communicate with the control loop over UDP:
- Port 20001: sensor data (sim → controller)
- Port 30010: joint commands (controller → sim)

### Timing
- Main control loop: 2 kHz (500 μs sleep)
- Policy runner thread: separate thread with 100 μs sleep
- Simulator timestep: 1 kHz (0.001 s)

### Third-Party Libraries (`third_party/`)
- `eigen/`: Header-only linear algebra (all math uses `VecXf`, `MatXf`, `Vec3f` Eigen types)
- `onnxruntime/`: ONNX inference (x86 and aarch64 binaries included)
- `Lite3_MotionSDK/`: Robot hardware SDK (platform-specific `.so`)
- `mujoco/`: MuJoCo physics engine (optional)
- `gamepad/`: Gamepad SDK wrapper

## Key Files for Common Tasks

| Task | File |
|------|------|
| Add a new state | Subclass `state_base.h`, register in `state_machine.hpp` |
| Change policy model | Replace `policy/ppo/policy.onnx`, update obs dims in `run_policy/lite3_test_policy_runner_onnx.cpp` |
| Add a new simulator | Implement `robot_interface.h` virtual methods |
| Tune safety limits | `rl_control_state_onnx.hpp` `PostureUnsafeCheck()` |
| Change control gains | `state_machine/parameters/` config files |
