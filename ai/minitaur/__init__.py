from gym.envs.registration import register

register(
    id='Minitaur-v0',
    entry_point='minitaur.minitaur:MinitaurEnv', #directory/python_file:constructor
    max_episode_steps=1000
)
