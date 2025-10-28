"""
Example 3: PyTorch Policy
===========================
This example demonstrates using a PyTorch model as a policy.
No ONNX conversion needed!
"""
import pylite3
import numpy as np
import time

try:
    import torch
    import torch.nn as nn
except ImportError:
    print("PyTorch not installed. Install with: pip install torch")
    exit(1)

class SimplePolicy(nn.Module):
    """Simple MLP policy for demonstration."""

    def __init__(self, obs_dim=45, act_dim=12, hidden_dim=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, act_dim),
        )

    def forward(self, x):
        return self.net(x)

class PyTorchPolicyWrapper:
    """Wrapper to use PyTorch model as Lite3 policy."""

    def __init__(self, model_path=None):
        self.model = SimplePolicy()

        if model_path:
            self.model.load_state_dict(torch.load(model_path))

        self.model.eval()

        # Configuration
        self.default_pose = np.array([0.0, -0.8, 1.6] * 4, dtype=np.float32)
        self.kp = np.ones(12, dtype=np.float32) * 30.0
        self.kd = np.ones(12, dtype=np.float32) * 1.0

    def __call__(self, state):
        """Policy inference."""
        # Prepare observation
        obs = np.concatenate([
            state.base_omega,
            state.projected_gravity,
            state.cmd_vel_normlized,
            state.joint_pos,
            state.joint_vel,
        ]).astype(np.float32)

        # Run PyTorch inference
        with torch.no_grad():
            obs_tensor = torch.from_numpy(obs).unsqueeze(0)
            action_tensor = self.model(obs_tensor)
            action = action_tensor.numpy().flatten()

        # Scale action and add to default pose
        joint_target = self.default_pose + action * 0.2

        # Create RobotAction
        robot_action = pylite3.RobotAction()
        robot_action.goal_joint_pos = joint_target
        robot_action.goal_joint_vel = np.zeros(12, dtype=np.float32)
        robot_action.kp = self.kp
        robot_action.kd = self.kd
        robot_action.tau_ff = np.zeros(12, dtype=np.float32)

        return robot_action

def main():
    print("=== Example 3: PyTorch Policy ===\n")

    # Create policy
    policy = PyTorchPolicyWrapper()
    print("PyTorch policy created (random weights for demo)")

    # Create robot
    robot = pylite3.Lite3Controller(use_sim=True)
    robot.initialize()

    # Stand up
    print("Standing up...")
    robot.stand_up()

    # Set velocity command
    robot.set_velocity(vx=0.3, vy=0.0, vyaw=0.0)

    # Set PyTorch policy
    print("Running PyTorch policy...")
    robot.set_custom_policy(policy, frequency=50.0)
    robot.run_async(pylite3.ControlMode.CUSTOM_CALLBACK)

    # Run for 10 seconds
    time.sleep(10.0)

    # Stop
    robot.stop()
    print("Example complete!")

if __name__ == "__main__":
    main()
