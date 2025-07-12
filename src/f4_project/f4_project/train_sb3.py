#! /usr/bin/env python3
import gymnasium as gym
from train_env_disp_mem import DroneGazeboEnv
# import time
# import numpy as np

from stable_baselines3 import TD3


    # Register the environment
gym.register(
    id='GazeboIrisEnv-v0',
    entry_point='train_env_disp_mem:DroneGazeboEnv', 
)

# env  = make_vec_env("GazeboIrisEnv-v0", n_envs=2, seed=0,vec_env_cls=DummyVecEnv)
env = gym.make("GazeboIrisEnv-v0")
# obs = env.reset()
# prev_time = time.time()
done = False

model = TD3.load("td3_run_laser_stacked", print_system_info=True, env=env)
# model = TD3("MultiInputPolicy", env, verbose=1,tensorboard_log="./tensorboard/td3/", learning_rate= 0.0002, learning_starts= 500)
for i in range(900):
    model.learn(total_timesteps=1000,tb_log_name="td3_run_laser_stacked",reset_num_timesteps=False)
    model.save("td3_run_laser_stacked")
# vec_env = model.get_env()


# obs = vec_env.reset()
# while True:
#     if(time.time() - prev_time > 0.001):
#         action, _states = model.predict(obs)
#         obs, rewards, dones, info = vec_env.step(action)
#         prev_time = time.time()
#      env.render("human")

    
