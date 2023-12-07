import sys

import gymnasium as gym
import mmEnv
from stable_baselines3 import PPO
import pandas as pd

env = gym.make("mapmatching-v0")
model = PPO.load("ppo_cartpole")
print("test")
for idx in range(0,2700):
    obs,_ = env.reset(options={'trace_idx':idx})
    while True:
        action, _ = model.predict(obs)
        obs, rewards, dones,_, info = env.step(action[0])
        if dones:
            break
env.save()