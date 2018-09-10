from gym.envs.registration import register

register(
    id='testenv-v0',
    entry_point='testenv.humanoid:HumanoidEnv', #directory/python_file:constructor
    max_episode_steps=1000
)
