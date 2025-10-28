"""
Unit tests for PyLite3 API
"""
import pytest
import numpy as np
import pylite3
import time


class TestBasicAPI:
    """Test basic API functionality."""

    def test_import(self):
        """Test that module imports correctly."""
        assert pylite3.__version__ is not None
        assert pylite3.version() is not None

    def test_create_controller(self):
        """Test controller creation."""
        robot = pylite3.Lite3Controller(use_sim=True)
        assert robot is not None

    def test_config(self):
        """Test configuration."""
        config = pylite3.ControllerConfig()
        assert config.control_frequency == 1000.0
        assert config.policy_frequency == 83.0

        config.control_frequency = 500.0
        assert config.control_frequency == 500.0

    def test_control_modes(self):
        """Test control mode enum."""
        assert pylite3.ControlMode.IDLE is not None
        assert pylite3.ControlMode.POSITION_CONTROL is not None
        assert pylite3.ControlMode.RL_CONTROL is not None
        assert pylite3.ControlMode.CUSTOM_CALLBACK is not None


class TestRobotState:
    """Test robot state structures."""

    def test_robot_basic_state(self):
        """Test RobotBasicState creation."""
        state = pylite3.RobotBasicState()
        assert state is not None

    def test_robot_action(self):
        """Test RobotAction creation and modification."""
        action = pylite3.RobotAction()
        assert action is not None

        # Set fields
        action.goal_joint_pos = np.zeros(12, dtype=np.float32)
        action.kp = np.ones(12, dtype=np.float32) * 30.0
        action.kd = np.ones(12, dtype=np.float32) * 1.0
        action.tau_ff = np.zeros(12, dtype=np.float32)

        assert action.goal_joint_pos.shape == (12,)
        assert action.kp.shape == (12,)


class TestControllerOperations:
    """Test controller operations."""

    @pytest.fixture
    def robot(self):
        """Create robot controller for testing."""
        robot = pylite3.Lite3Controller(use_sim=True)
        yield robot
        if robot.is_running():
            robot.stop()

    def test_initialize(self, robot):
        """Test initialization."""
        robot.initialize()
        # Should not raise exception

    def test_set_velocity(self, robot):
        """Test velocity setting."""
        robot.set_velocity(0.5, 0.2, -0.3)
        cmd_vel = robot.get_command_velocities()

        np.testing.assert_almost_equal(cmd_vel[0], 0.5, decimal=3)
        np.testing.assert_almost_equal(cmd_vel[1], 0.2, decimal=3)
        np.testing.assert_almost_equal(cmd_vel[2], -0.3, decimal=3)

    def test_velocity_clamping(self, robot):
        """Test velocity clamping to [-1, 1]."""
        robot.set_velocity(2.0, -3.0, 0.5)
        cmd_vel = robot.get_command_velocities()

        assert cmd_vel[0] == 1.0   # Clamped to 1.0
        assert cmd_vel[1] == -1.0  # Clamped to -1.0
        assert cmd_vel[2] == 0.5   # Within range

    def test_set_default_pose(self, robot):
        """Test setting default pose."""
        new_pose = np.array([0.1, -0.9, 1.7] * 4, dtype=np.float32)
        robot.set_default_pose(new_pose)

        config = robot.get_config()
        np.testing.assert_array_almost_equal(config.default_pose, new_pose)

    def test_set_safety_limits(self, robot):
        """Test setting safety limits."""
        robot.set_safety_limits(35.0, 50.0)

        config = robot.get_config()
        assert config.max_roll_deg == 35.0
        assert config.max_pitch_deg == 50.0

    def test_custom_policy(self, robot):
        """Test custom policy callback."""
        callback_called = [False]

        def test_policy(state):
            callback_called[0] = True
            action = pylite3.RobotAction()
            action.goal_joint_pos = np.zeros(12, dtype=np.float32)
            action.kp = np.ones(12, dtype=np.float32) * 30.0
            action.kd = np.ones(12, dtype=np.float32) * 1.0
            return action

        robot.set_custom_policy(test_policy, 100.0)
        # Should not raise exception

class TestUtilities:
    """Test utility functions."""

    def test_create_default_action(self):
        """Test create_default_action utility."""
        action = pylite3.utils.create_default_action(kp=40.0, kd=2.0)

        assert action.goal_joint_pos.shape == (12,)
        assert action.kp.shape == (12,)
        assert np.all(action.kp == 40.0)
        assert np.all(action.kd == 2.0)

    def test_state_to_numpy(self):
        """Test state_to_numpy utility."""
        state = pylite3.RobotBasicState()
        # Note: state fields might be uninitialized, but function should work
        obs = pylite3.utils.state_to_numpy(state)
        assert obs.shape == (45,)

    def test_get_default_standing_pose(self):
        """Test get_default_standing_pose utility."""
        pose = pylite3.utils.get_default_standing_pose()
        assert pose.shape == (12,)
        expected = np.array([0.0, -0.8, 1.6] * 4, dtype=np.float32)
        np.testing.assert_array_equal(pose, expected)


class TestGymEnvironment:
    """Test Gymnasium environment."""

    @pytest.fixture
    def env(self):
        """Create Gym environment."""
        env = pylite3.Lite3GymEnv(use_sim=True, max_episode_steps=100)
        yield env
        env.close()

    def test_env_creation(self, env):
        """Test environment creation."""
        assert env is not None
        assert env.observation_space.shape == (45,)
        assert env.action_space.shape == (12,)

    def test_reset(self, env):
        """Test environment reset."""
        obs, info = env.reset()
        assert obs.shape == (45,)
        assert isinstance(info, dict)

    def test_step(self, env):
        """Test environment step."""
        env.reset()
        action = np.zeros(12, dtype=np.float32)
        obs, reward, terminated, truncated, info = env.step(action)

        assert obs.shape == (45,)
        assert isinstance(reward, (int, float))
        assert isinstance(terminated, bool)
        assert isinstance(truncated, bool)
        assert isinstance(info, dict)


def test_context_manager():
    """Test using controller as context manager."""
    with pylite3.Lite3Controller(use_sim=True) as robot:
        robot.set_velocity(0.5, 0.0, 0.0)
    # Should automatically stop on exit


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
