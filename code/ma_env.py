import gym
import numpy as np
from gym import envs
import matplotlib.pyplot as plt
import seaborn as sns
import time
import make_env
sns.set()
#import gym_minigrid.envs
#from gym_minigrid.wrappers import *

import gym
from gym import envs
import numpy as np
import matplotlib.pyplot as plt
import time
import seaborn as sns
sns.set()

env =make_env.make_env('simple_spread')
obs =env.reset()

print(obs, len(obs))
print(env.observation_space, env.action_space, env.n)

env.render()
time.sleep(10)

#end of code