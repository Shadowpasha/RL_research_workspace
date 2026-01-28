import numpy as np
import torch
import gymnasium as gym
import argparse
import os
import TD3
from train_env_disp_mem import DroneGazeboEnv
import random
gym.register(
    id='GazeboIrisEnv-v0',
    entry_point='train_env_disp_mem:DroneGazeboEnv', 
)

# Runs policy for X episodes and returns average reward
# A fixed seed is used for the eval environment
def eval_policy(policy, eval_env, seed, eval_episodes=10):
    # eval_env.seed(seed + 100)
    
    avg_reward = 0.
    success_rate = 0.
    
    for i in range(eval_episodes):
        state, info = eval_env.reset()
        done = False
        episode_reward = 0
        while not done:
            action = policy.select_action(np.array(state))
            state, reward, done, truncated, info = eval_env.step(action)
            episode_reward += reward
        
        print(f"Episode {i+1}: Reward = {episode_reward:.3f}, Reached = {info.get('reached', False)}")
        avg_reward += episode_reward
        if info.get('reached', False):
            success_rate += 1.0

    avg_reward /= eval_episodes
    success_rate /= eval_episodes

    print("---------------------------------------")
    print(f"Evaluation over {eval_episodes} episodes: {avg_reward:.3f}")
    print(f"Success Rate: {success_rate * 100:.2f}%")
    print("---------------------------------------")
    return avg_reward, success_rate


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", default="TD3")                  # Policy name (TD3, DDPG or OurDDPG)
    parser.add_argument("--env", default="GazeboIrisEnv-v0")        # OpenAI gym environment name
    parser.add_argument("--seed", default=0, type=int)              # Sets Gym, PyTorch and Numpy seeds
    parser.add_argument("--load_model", default="normal_21_01_2026_13_23")  # Model load file name, required
    parser.add_argument("--episodes", default=10, type=int)         # Number of episodes to evaluate
    args = parser.parse_args()
    
    print("---------------------------------------")
    print(f"Settings: Policy: {args.policy}, Env: {args.env}, Seed: {args.seed}, Model: {args.load_model}")
    print("---------------------------------------")

    if not os.path.exists("./models"):
        print("Error: ./models directory does not exist. Cannot load model.")
        exit()

    env = gym.make(args.env)

    # Set seeds
    # env.seed(args.seed)
    # env.action_space.seed(args.seed)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    
    state_dim = 70
    action_dim = 2
    max_action = 1.0

    kwargs = {
        "state_dim": state_dim,
        "action_dim": action_dim,
        "max_action": max_action,
        "discount": 0.99,
        "tau": 0.005,
    }

    # Initialize policy
    if args.policy == "TD3":
        # Target policy smoothing is scaled wrt the action scale
        kwargs["policy_noise"] = 0.2 * max_action
        kwargs["noise_clip"] = 0.5 * max_action
        kwargs["policy_freq"] = 2
        policy = TD3.TD3(**kwargs)

    # Load the specified model
    policy.load(f"models/{args.load_model}")
    print(f"Model loaded from models/{args.load_model}")
    
    eval_policy(policy, env, args.seed, eval_episodes=args.episodes)
