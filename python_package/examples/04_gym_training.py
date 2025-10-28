"""
Example 4: Gymnasium Integration
==================================
This example shows how to use the Lite3 Gym environment for RL training.
"""
import pylite3
import numpy as np

try:
    import gymnasium as gym
except ImportError:
    print("gymnasium not installed. Install with: pip install gymnasium")
    exit(1)

def random_policy_demo():
    """Demo with random actions."""
    print("=== Gymnasium Environment Demo ===\n")

    # Create environment
    env = pylite3.Lite3GymEnv(
        use_sim=True,
        max_episode_steps=500,
        control_frequency=50.0,
    )

    print(f"Observation space: {env.observation_space}")
    print(f"Action space: {env.action_space}\n")

    # Run a few episodes
    num_episodes = 3

    for episode in range(num_episodes):
        obs, info = env.reset()
        episode_reward = 0.0

        print(f"Episode {episode + 1} starting...")

        for step in range(100):
            # Random action
            action = env.action_space.sample()

            # Step environment
            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward

            if terminated or truncated:
                break

        print(f"  Episode {episode + 1} finished: "
              f"Steps={step + 1}, Reward={episode_reward:.2f}\n")

    env.close()
    print("Demo complete!")

def train_with_stable_baselines3():
    """
    Example of training with Stable-Baselines3 (if installed).

    Install with: pip install stable-baselines3
    """
    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.env_checker import check_env
    except ImportError:
        print("\nStable-Baselines3 not installed.")
        print("Install with: pip install stable-baselines3")
        return

    print("\n=== Training with Stable-Baselines3 ===\n")

    # Create environment
    env = pylite3.Lite3GymEnv(use_sim=True)

    # Check environment
    print("Checking environment...")
    check_env(env)
    print("Environment check passed!\n")

    # Create PPO agent
    print("Creating PPO agent...")
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
    )

    # Train
    print("\nTraining for 10,000 steps...")
    model.learn(total_timesteps=10000)

    # Save model
    model.save("lite3_ppo")
    print("\nModel saved to: lite3_ppo.zip")

    # Test trained policy
    print("\nTesting trained policy...")
    obs, info = env.reset()
    for _ in range(200):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            break

    env.close()
    print("Training example complete!")

if __name__ == "__main__":
    # Run random policy demo
    random_policy_demo()

    # Uncomment to run SB3 training
    # train_with_stable_baselines3()
