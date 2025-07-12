import numpy as np
import gymnasium as gym
import argparse
from datetime import datetime
import TD3
from torch.utils.tensorboard import SummaryWriter
from train_env_disp_mem_px4 import DroneGazeboEnv


gym.register(
    id='GazeboIrisEnv-v0',
    entry_point='train_env_disp_mem_px4:DroneGazeboEnv', 
)

if __name__ == "__main__":
	
	parser = argparse.ArgumentParser()
	parser.add_argument("--load_model", default="2d_new_28_10_2024_09_36") 
	args = parser.parse_args()

	env = gym.make("GazeboIrisEnv-v0")

	kwargs = {
		"state_dim": 34,
		"action_dim": 2,
		"max_action": 1,
		"discount": 0.99,
		"tau": 0.005,
	}

	policy = TD3.TD3(**kwargs)

	if args.load_model != "":
		policy_file = args.load_model
		policy.load(f"models/{policy_file}")
	
	done = False
	state, info = env.reset()
	episode_counter = 0

	for t in range(10000):
		

		# Select action according to policy
		action = policy.select_action(state,)

		# Perform action
		next_state, reward, done, truncated, info = env.step(action)

		state = next_state
		
		if done:
			done = False
			state, info = env.reset()
			episode_counter += 1
			if(episode_counter >= 20):
				break
