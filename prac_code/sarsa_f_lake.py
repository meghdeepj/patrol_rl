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
n_ep=5000
r_tot=0.
q = np.zeros([env.nS, env.nA])

print(q)

def update_q(prev_state, action, reward, next_state, next_action):
	q[prev_state][action]+=alpha*(reward+(gamma*q[next_state][next_action])-q[prev_state][action])

def ept_greed(state,epsilon):
	rand = np.random.random(1)[0]
	if rand >= epsilon:
		if np.min(q[state]) == np.max(q[state]):
			return env.action_space.sample()
		else:
			return np.argmax(q[state])
	else:
		return env.action_space.sample()
c_re=[]
ep=[]
for i in range(n_ep):
	r=0
	epsilon=1/count
	prev_state=env.reset()
	print('::::::::::::::episode no.: ', i, '::::::::::::::')
	env.render()
	action = ept_greed(prev_state,epsilon)
	while True:
		next_state, reward, done, info = env.step(action)
		next_action=ept_greed(next_state,epsilon)
		update_q(prev_state, action, reward, next_state, next_action)
		prev_state=next_state
		action=next_action
		env.render()
		if done:
			break
		#time.sleep(0.20)
	r+=reward
	count+=0.02
	print('episode reward: ', r)
	r_tot+=r
	ep.append(i)
	c_re.append(r_tot/(i+1))
print('total reward: ', r_tot)
print('avg reward :', r_tot/n_ep)
print('final Q: ', q)
plt.plot(ep,c_re)
plt.title("SARSA on FrozenLake-v0")
plt.ylim(0,1)
plt.xlabel("No. of iterations")
plt.ylabel("Cumulative Average Reward")
plt.show()
env.close()
#end of code