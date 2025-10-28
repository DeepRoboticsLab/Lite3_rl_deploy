"""
Utility functions for PyLite3
"""
import numpy as np


def create_default_action(kp=30.0, kd=1.0):
    """
    Create a default RobotAction with common gains.

    Args:
        kp (float): Proportional gain (default: 30.0)
        kd (float): Derivative gain (default: 1.0)

    Returns:
        RobotAction: Action with zero positions/velocities and specified gains
    """
    from . import RobotAction

    action = RobotAction()
    action.goal_joint_pos = np.zeros(12, dtype=np.float32)
    action.goal_joint_vel = np.zeros(12, dtype=np.float32)
    action.kp = np.ones(12, dtype=np.float32) * kp
    action.kd = np.ones(12, dtype=np.float32) * kd
    action.tau_ff = np.zeros(12, dtype=np.float32)
    return action


def state_to_numpy(state):
    """
    Convert RobotBasicState to a flat numpy array for ML applications.

    Args:
        state (RobotBasicState): Robot state

    Returns:
        numpy.ndarray: Flattened state vector (45-dim by default)
    """
    return np.concatenate([
        state.base_omega,
        state.projected_gravity,
        state.cmd_vel_normlized,
        state.joint_pos,
        state.joint_vel,
    ])


def numpy_to_action(action_array, kp=30.0, kd=1.0):
    """
    Convert numpy array to RobotAction.

    Args:
        action_array (numpy.ndarray): Joint positions (12-dim)
        kp (float or numpy.ndarray): Proportional gains
        kd (float or numpy.ndarray): Derivative gains

    Returns:
        RobotAction: Robot action
    """
    from . import RobotAction

    action = RobotAction()
    action.goal_joint_pos = np.array(action_array, dtype=np.float32)
    action.goal_joint_vel = np.zeros(12, dtype=np.float32)

    if isinstance(kp, (int, float)):
        action.kp = np.ones(12, dtype=np.float32) * kp
    else:
        action.kp = np.array(kp, dtype=np.float32)

    if isinstance(kd, (int, float)):
        action.kd = np.ones(12, dtype=np.float32) * kd
    else:
        action.kd = np.array(kd, dtype=np.float32)

    action.tau_ff = np.zeros(12, dtype=np.float32)
    return action


def get_default_standing_pose():
    """
    Get the default standing pose for Lite3.

    Returns:
        numpy.ndarray: Default joint positions (12-dim)
    """
    return np.array([0.0, -0.8, 1.6] * 4, dtype=np.float32)
