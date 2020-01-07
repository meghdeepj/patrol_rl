# mini grid q-learn implement

import gym
import numpy as np
from gym import envs
import matplotlib.pyplot as plt
import seaborn as sns
import time
sns.set()
import gym_minigrid.envs
from gym_minigrid.wrappers import *

env =gym.make('MiniGrid-Empty-5x5-v0')
#env= FlatObsWrapper(env)
obs = env.reset()
#print(env.nS)
#print(env.nA)
print(obs, len(obs), type(obs))
print(env.action_space)
print(env.observation_space)
#env.render()
#time.sleep(5)
env.close()