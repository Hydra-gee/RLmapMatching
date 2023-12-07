import gymnasium as gym
import mmEnv
import random
env = gym.make("mapmatching-v0")

obs = env.reset(options={'trace_idx':924})
while True:
    action = random.randint(0,1)
    obs, rewards, dones,_, info = env.step(action)
    if dones:
        break