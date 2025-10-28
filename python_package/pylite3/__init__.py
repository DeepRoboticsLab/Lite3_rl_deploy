"""
PyLite3 - Python API for Lite3 Quadruped Robot Control

This package provides a high-level Python interface for controlling the
DeepRobotics Lite3 quadruped robot in both simulation and hardware modes.

Example:
    >>> import pylite3
    >>> import numpy as np
    >>>
    >>> # Create controller
    >>> robot = pylite3.Lite3Controller(use_sim=True)
    >>>
    >>> # Stand up
    >>> robot.stand_up()
    >>>
    >>> # Walk forward
    >>> robot.set_velocity(0.5, 0.0, 0.0)
    >>> robot.run(pylite3.ControlMode.POSITION_CONTROL)
"""

__version__ = "1.0.0"
__author__ = "DeepRobotics & Claude Code"

# Import the compiled C++ extension
try:
    from .pylite3 import (
        Lite3Controller,
        ControlMode,
        RobotBasicState,
        RobotAction,
        ControllerConfig,
        version,
        create_default_config,
    )
except ImportError as e:
    import warnings
    warnings.warn(
        f"Failed to import pylite3 C++ extension: {e}\n"
        "Make sure you have built the C++ library with:\n"
        "  mkdir build && cd build\n"
        "  cmake .. -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON -DBUILD_PYTHON_BINDINGS=ON\n"
        "  make -j\n"
        "Then install the Python package with:\n"
        "  cd python_package && pip install -e ."
    )
    raise

# Import Python utilities
from .utils import create_default_action, state_to_numpy
from .gym_env import Lite3GymEnv

__all__ = [
    "Lite3Controller",
    "ControlMode",
    "RobotBasicState",
    "RobotAction",
    "ControllerConfig",
    "Lite3GymEnv",
    "create_default_action",
    "state_to_numpy",
    "version",
    "create_default_config",
]
