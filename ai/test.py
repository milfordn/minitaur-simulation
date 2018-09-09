import mujoco_py
import gym
import time
import minitaur.minitaur

env = gym.make('Minitaur-v0')
for i in range(200):
    observation = env.reset()
    #space = gym.space.Discrete()
    for t in range(1000):
        env.render()
        #print(observation)
        action = env.action_space.sample()
        #get action from CPGController
        observation, reward, done, _ = env.step(action)
        if done:
            #print("Episode finished.")
            break
