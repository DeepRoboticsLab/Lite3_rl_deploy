# Build Guide for Lite3 API and Python Bindings

This guide provides step-by-step instructions for building the Lite3 C++ API wrapper and Python bindings.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Quick Build](#quick-build)
3. [Detailed Build Instructions](#detailed-build-instructions)
4. [Testing](#testing)
5. [Python Package Installation](#python-package-installation)
6. [Troubleshooting](#troubleshooting)

## Prerequisites

### System Requirements

- **OS**: Ubuntu 20.04+ (for simulation) or ARM Linux (for hardware)
- **Compiler**: GCC 9+ or Clang 10+
- **CMake**: 3.10+
- **Python**: 3.8+

### Dependencies

#### Ubuntu/Debian

```bash
# Core dependencies
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-dev \
    libdw-dev \
    libeigen3-dev

# Python dependencies
pip3 install --upgrade pip
pip3 install numpy "numpy<2.0" pybullet pybind11 matplotlib pandas
```

#### Python Packages

```bash
pip3 install pybind11 numpy pybullet

# Optional: for RL training
pip3 install gymnasium torch stable-baselines3

# Optional: for visualization
pip3 install matplotlib pandas h5py meshcat
```

## Quick Build

```bash
# Clone repository with submodules
git clone --recurse-submodules https://github.com/DeepRoboticsLab/Lite3_rl_deploy.git
cd Lite3_rl_deploy

# Build everything (simulation + API + Python bindings)
./scripts/build_all.sh

# Install Python package
cd python_package
pip3 install -e .

# Test installation
python3 -c "import pylite3; print(pylite3.version())"
```

## Detailed Build Instructions

### Step 1: Clone Repository

```bash
git clone --recurse-submodules https://github.com/DeepRoboticsLab/Lite3_rl_deploy.git
cd Lite3_rl_deploy
```

If you forgot `--recurse-submodules`:
```bash
git submodule update --init --recursive
```

### Step 2: Create Build Directory

```bash
mkdir -p build
cd build
```

### Step 3: Configure with CMake

#### For Simulation (x86)

```bash
cmake .. \
    -DBUILD_PLATFORM=x86 \
    -DBUILD_SIM=ON \
    -DSEND_REMOTE=OFF \
    -DBUILD_LITE3_API=ON \
    -DBUILD_PYTHON_BINDINGS=ON
```

#### For Hardware (ARM)

```bash
cmake .. \
    -DBUILD_PLATFORM=arm \
    -DBUILD_SIM=OFF \
    -DSEND_REMOTE=OFF \
    -DBUILD_LITE3_API=ON \
    -DBUILD_PYTHON_BINDINGS=ON
```

### Step 4: Build

```bash
make -j$(nproc)
```

This will build:
- `rl_deploy` - Original executable
- `liblite3_api.so` - C++ API library
- `test_lite3_controller` - Unit tests
- `simple_example` - C++ example
- `pylite3.so` - Python extension module

### Step 5: Verify Build

```bash
# Run C++ tests
./test_lite3_controller

# Check Python module was built
ls ../python_package/pylite3/pylite3*.so
```

## Testing

### C++ Tests

```bash
cd build
./test_lite3_controller
```

### C++ Examples

```bash
# Terminal 1: Start simulator
cd interface/robot/simulation
python3 pybullet_simulation.py

# Terminal 2: Run example
cd build
./simple_example
```

### Python Tests

```bash
cd python_package
python3 -m pytest tests/
```

### Python Examples

```bash
# Terminal 1: Start simulator (if not already running)
cd interface/robot/simulation
python3 pybullet_simulation.py

# Terminal 2: Run examples
cd python_package/examples
python3 01_basic_control.py
python3 02_custom_policy.py   # test thu dung khong vung va khong di chuyen bang phim duoc
python3 03_pytorch_policy.py
```

## Python Package Installation

### Development Mode (Recommended)

Install in editable mode for development:

```bash
cd python_package
pip3 install -e .
```

Changes to Python files will be reflected immediately without reinstalling.

### Standard Installation

```bash
cd python_package
pip3 install .
```

### With Optional Dependencies

```bash
# Install with Gymnasium support
pip3 install -e ".[gym]"

# Install with PyTorch support
pip3 install -e ".[torch]"

# Install with visualization tools
pip3 install -e ".[viz]"

# Install everything
pip3 install -e ".[all]"
```

## Build Options

### CMake Options

| Option | Values | Default | Description |
|--------|--------|---------|-------------|
| `BUILD_PLATFORM` | `x86`, `arm` | `x86` | Target platform |
| `BUILD_SIM` | `ON`, `OFF` | `OFF` | Enable simulation |
| `SEND_REMOTE` | `ON`, `OFF` | `OFF` | Auto-deploy to robot |
| `USE_MJCPP` | `ON`, `OFF` | `OFF` | Use MuJoCo C++ |
| `BUILD_LITE3_API` | `ON`, `OFF` | `ON` | Build API wrapper |
| `BUILD_PYTHON_BINDINGS` | `ON`, `OFF` | `ON` | Build Python bindings |

### Example Configurations

#### Simulation with Python

```bash
cmake .. -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON -DBUILD_PYTHON_BINDINGS=ON
```

#### Hardware with Python

```bash
cmake .. -DBUILD_PLATFORM=arm -DBUILD_SIM=OFF -DBUILD_PYTHON_BINDINGS=ON
```

#### C++ API Only (No Python)

```bash
cmake .. -DBUILD_LITE3_API=ON -DBUILD_PYTHON_BINDINGS=OFF
```

## Troubleshooting

### pybind11 Not Found

```
CMake Error: Could not find pybind11
```

**Solution:**
```bash
pip3 install pybind11
# Or install system package
sudo apt-get install pybind11-dev
```

### Python Module Import Error

```python
ImportError: cannot import name 'pylite3'
```

**Solution:**
1. Check that `pylite3*.so` was built:
   ```bash
   find . -name "pylite3*.so"
   ```

2. Ensure it's in the correct location:
   ```bash
   ls python_package/pylite3/pylite3*.so
   ```

3. Reinstall Python package:
   ```bash
   cd python_package
   pip3 install -e . --force-reinstall
   ```

### Eigen Not Found

```
Could not find Eigen3
```

**Solution:**
```bash
sudo apt-get install libeigen3-dev
```

### ONNX Runtime Not Found

The ONNX runtime should be in `third_party/onnxruntime/`. If missing:

```bash
# Check third_party directory
ls third_party/onnxruntime/
```

If it's missing, you may need to download it separately or check the git submodules.

### Simulation Not Starting

**Issue:** Python bindings work but robot doesn't move in simulation.

**Solution:**
1. Make sure PyBullet simulation is running in a separate terminal:
   ```bash
   cd interface/robot/simulation
   python3 pybullet_simulation.py
   ```

2. Check network connectivity (simulation uses UDP):
   ```bash
   netstat -an | grep 20001
   netstat -an | grep 30010
   ```

### Build Fails on ARM Platform

If cross-compiling for ARM on x86 machine:

```bash
# Install ARM cross-compiler
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# Build
cmake .. -DBUILD_PLATFORM=arm
make -j
```

## Performance Optimization

### Release Build

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SIM=ON -DBUILD_PYTHON_BINDINGS=ON
make -j
```

### Link-Time Optimization

```bash
cmake .. -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON
make -j
```

### Parallel Build

```bash
make -j$(nproc)  # Use all CPU cores
```

## Clean Build

To start fresh:

```bash
cd Lite3_rl_deploy
rm -rf build
mkdir build
cd build
cmake .. [your options]
make -j
```

## Hardware Deployment

### Building on Robot

```bash
# SSH to robot
ssh ysc@192.168.2.1  # Password: ' or 123456 or firefly

# Build on robot
cd Lite3_rl_deploy
mkdir build && cd build
cmake .. -DBUILD_PLATFORM=arm -DBUILD_SIM=OFF -DBUILD_PYTHON_BINDINGS=ON
make -j

# Install Python package
cd ../python_package
pip3 install -e .
```

### Cross-Compiling

For cross-compilation from x86 to ARM, you'll need an ARM toolchain and may need to adjust CMake settings for cross-compilation.

## Next Steps

After successful build:

1. **Run Tests**: Verify everything works
   ```bash
   cd build
   ./test_lite3_controller
   ```

2. **Try Examples**: Run Python examples
   ```bash
   cd python_package/examples
   python3 01_basic_control.py
   ```

3. **Develop Your Algorithm**: Start implementing your custom policy!

## Support

If you encounter issues not covered here:

- Check [GitHub Issues](https://github.com/DeepRoboticsLab/Lite3_rl_deploy/issues)
- Join [Discord Community](https://discord.gg/gdM9mQutC8)
- Contact DeepRobotics support

## License

MIT License - see LICENSE file for details.
