import sys

import gymnasium as gym
import mmEnv
from stable_baselines3 import PPO, DQN
from stable_baselines3.common.env_util import make_vec_env

# Parallel environments
# vec_env = make_vec_env("mapmatching-v0", n_envs=12)
env = gym.make("mapmatching-v0")
# model = PPO("MlpPolicy", env, verbose=1)
model = PPO.load("ppo_cartpole",env=env)
model.learn(total_timesteps=100000)
model.save("ppo_cartpole")
env.save()
print("end")
sys.exit()
#
# del model # remove to demonstrate saving and loading
#
# model = PPO.load("ppo_cartpole")
#
# print("test")
# obs = env.reset(options={'trace_idx':20})
# while True:
#     action, _ = model.predict(obs)
#     obs, rewards, dones,_, info = env.step(action)