import gymnasium as gym
from train_env_disp_mem import DroneGazeboEnv

gym.register(
    id='GazeboIrisEnv-v0',
    entry_point='train_env_disp_mem:DroneGazeboEnv', 
)

from stable_baselines3 import TD3

env = gym.make('GazeboIrisEnv-v0')

model = TD3("MlpPolicy", env, verbose=1, tensorboard_log="./stable_runs/one_slice_camera/")
model.learn(total_timesteps=150000)
model.save("one_slice_camera")

	
	
