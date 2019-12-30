#Q-Learning python implement /w OpenAI Gym
#Q-Learning algorithm on random walk
import gym
import pandas as pd
import time
import matplotlib.pyplot as plt
from gym import envs
#import random
import numpy as np
import seaborn as sns
sns.set()
from IPython.core.debugger import set_trace

#from src.rlenvs.rlenvs.envs.gridworld import GridworldEnv
env=gym.make('FrozenLake-v0', is_slippery=False)

#env=GridworldEnv()

print('State space: ', env.nS, "\nAction space: ", env.nA)

gamma=0.95
alpha=0.4
count=1.0
n_ep=2000
r_tot=0.
q = np.zeros([env.nS, env.nA])

print(q)

def update_q(prev_state, action, reward, next_state):
	qa=np.max(q[next_state])
	q[prev_state][action]+=alpha*(reward+(gamma*qa)-q[prev_state][action])

def ept_greed(state,epsilon):
	rand = np.random.random(1)[0]
	if rand >= epsilon:
		if np.min(q[state]) == np.max(q[state]):
			return env.action_space.sample()
		else:
			return np.argmax(q[state])
	else:
		return env.action_space.sample()

for i in range(n_ep):
	r=0
	epsilon=1/count
	prev_state=env.reset()
	print('episode no.: ', i)
	env.render()
	while True:
		action = ept_greed(prev_state,epsilon)
		next_state, reward, done, info = env.step(action)
		update_q(prev_state, action, reward,next_state)
		prev_state=next_state
		env.render()
		if done:
			break
		#ime.sleep(0.05)
	r+=reward
	count+=0.01
	print('episode reward: ', r)
	r_tot+=r
print('total reward: ', r_tot)
print('avg reward :', r_tot/n_ep)
print('final Q: ', q)
env.close()
#end of code