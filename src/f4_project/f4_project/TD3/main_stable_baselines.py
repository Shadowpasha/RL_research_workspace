import gymnasium as gym
from train_env_disp_mem import DroneGazeboEnv

gym.register(
    id='GazeboIrisEnv-v0',
    entry_point='train_env_disp_mem_CNN:DroneGazeboEnv', 
)

from stable_baselines3 import TD3

env = gym.make('GazeboIrisEnv-v0')

model = TD3("MultiInputPolicy", env, verbose=1, tensorboard_log="./stable_runs/full_camera/",learning_starts=2000, 
                                batch_size=128, train_freq=(200, 'step'), gradient_steps=200,learning_rate=5e-4)
model.learn(total_timesteps=300000)
model.save("full_camera")

	
	
