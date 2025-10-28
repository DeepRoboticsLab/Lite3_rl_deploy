"""
Gymnasium environment wrapper for Lite3 robot
"""
import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    raise ImportError(
        "gymnasium not installed. Install with: pip install gymnasium"
    )

from . import Lite3Controller, ControlMode, RobotAction
from .utils import state_to_numpy


class Lite3GymEnv(gym.Env):
    """
    Gymnasium environment for Lite3 quadruped robot.

    This environment can be used with standard RL libraries like
    Stable-Baselines3, RLlib, etc.

    Observation Space (45-dim):
        - Base angular velocity (3)
        - Projected gravity (3)
        - Command velocity (3)
        - Joint positions (12)
        - Joint velocities (12)
        - Last action (12)

    Action Space (12-dim):
        - Joint position targets [-1, 1] (normalized)

    Example:
        >>> env = Lite3GymEnv(use_sim=True)
        >>> obs, info = env.reset()
        >>> for _ in range(1000):
        ...     action = env.action_space.sample()
        ...     obs, reward, terminated, truncated, info = env.step(action)
        ...     if terminated or truncated:
        ...         obs, info = env.reset()
    """

    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(
        self,
        use_sim=True,
        max_episode_steps=1000,
        control_frequency=50.0,
        reward_weights=None,
    ):
        """
        Initialize Lite3 Gym environment.

        Args:
            use_sim (bool): Use simulation or hardware
            max_episode_steps (int): Maximum steps per episode
            control_frequency (float): Control frequency in Hz
            reward_weights (dict): Weights for reward components
        """
        super().__init__()

        # Create robot controller
        self.robot = Lite3Controller(use_sim=use_sim)
        self.robot.set_policy_frequency(control_frequency)
        self.robot.initialize()

        # Episode management
        self.max_episode_steps = max_episode_steps
        self.current_step = 0

        # Observation space: 45-dim
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(45,),
            dtype=np.float32,
        )

        # Action space: 12-dim normalized joint positions
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(12,),
            dtype=np.float32,
        )

        # Reward weights
        if reward_weights is None:
            self.reward_weights = {
                "forward_velocity": 1.0,
                "energy": -0.01,
                "orientation": -0.5,
                "joint_limits": -1.0,
            }
        else:
            self.reward_weights = reward_weights

        # State tracking
        self.last_action = np.zeros(12, dtype=np.float32)
        self.default_pose = np.array([0.0, -0.8, 1.6] * 4, dtype=np.float32)

    def reset(self, seed=None, options=None):
        """Reset the environment."""
        super().reset(seed=seed)

        # Stand up
        self.robot.stand_up(duration=2.0, blocking=True)

        # Reset episode variables
        self.current_step = 0
        self.last_action = np.zeros(12, dtype=np.float32)

        # Set random command velocity if specified
        if options and "target_velocity" in options:
            vx, vy, vyaw = options["target_velocity"]
        else:
            vx, vy, vyaw = 0.5, 0.0, 0.0

        self.robot.set_velocity(vx, vy, vyaw)

        # Get initial observation
        state = self.robot.get_state()
        obs = self._state_to_obs(state)

        info = {}
        return obs, info

    def step(self, action):
        """Take a step in the environment."""
        # Denormalize action from [-1, 1] to joint limits
        action = np.clip(action, -1.0, 1.0)
        joint_pos_target = self.default_pose + action * 0.5  # Scale by 0.5 rad

        # Send action to robot
        robot_action = RobotAction()
        robot_action.goal_joint_pos = joint_pos_target
        robot_action.kp = np.ones(12, dtype=np.float32) * 30.0
        robot_action.kd = np.ones(12, dtype=np.float32) * 1.0
        robot_action.tau_ff = np.zeros(12, dtype=np.float32)

        self.robot.set_joint_command(
            robot_action.goal_joint_pos,
            robot_action.kp,
            robot_action.kd,
            robot_action.goal_joint_vel,
            robot_action.tau_ff,
        )

        # Wait for control step
        import time
        time.sleep(1.0 / 50.0)  # 50 Hz

        # Get new state
        state = self.robot.get_state()
        obs = self._state_to_obs(state)

        # Compute reward
        reward = self._compute_reward(state, action)

        # Check termination
        terminated = not self.robot.is_safe()
        truncated = self.current_step >= self.max_episode_steps

        self.current_step += 1
        self.last_action = action

        info = {
            "step": self.current_step,
            "is_safe": self.robot.is_safe(),
        }

        return obs, reward, terminated, truncated, info

    def _state_to_obs(self, state):
        """Convert robot state to observation."""
        obs = np.concatenate([
            state.base_omega,
            state.projected_gravity,
            state.cmd_vel_normlized,
            state.joint_pos,
            state.joint_vel,
            self.last_action,
        ]).astype(np.float32)
        return obs

    def _compute_reward(self, state, action):
        """Compute reward based on current state and action."""
        reward = 0.0

        # Forward velocity reward
        cmd_vel = state.cmd_vel_normlized
        # Assuming forward progress (simplified)
        reward += self.reward_weights["forward_velocity"] * cmd_vel[0]

        # Energy penalty (joint velocities and torques)
        energy = np.sum(np.abs(state.joint_vel)) + np.sum(np.abs(state.joint_tau)) * 0.01
        reward += self.reward_weights["energy"] * energy

        # Orientation penalty
        rpy = state.base_rpy
        orientation_error = np.abs(rpy[0]) + np.abs(rpy[1])  # Roll + pitch
        reward += self.reward_weights["orientation"] * orientation_error

        # Joint limits penalty
        joint_limits_violation = np.sum(np.maximum(0, np.abs(state.joint_pos) - 2.5))
        reward += self.reward_weights["joint_limits"] * joint_limits_violation

        return reward

    def close(self):
        """Clean up resources."""
        self.robot.stop()

    def render(self):
        """Render the environment (handled by simulator)."""
        pass
