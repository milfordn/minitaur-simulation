import mujoco_py
import gym
import time
import testenv.humanoid

env = gym.make('testenv-v0')
for i in range(200):
    observation = env.reset()
    for t in range(1000):
        env.render()
        #print(observation)
        action = env.action_space.sample()
        observation, reward, done, _ = env.step(action)
        if done:
            #print("Episode finished.")
            break
