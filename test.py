import gymnasium as gym
import numpy as np
from gymnasium.wrappers import RecordVideo

from segway_sim.envs import SegwayEnv
# env = gym.make("InvertedPendulum-v4", render_mode="rgb_array")

env =  SegwayEnv(render_mode="rgb_array")
env = RecordVideo(env, video_folder="videos", name_prefix="segway")
observation, info = env.reset(seed=42)
print(env.action_space)
for _ in range(500):
    action = env.action_space.sample()
    observation, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        observation, info = env.reset()
env.close()