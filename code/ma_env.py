#Mixed Coop-Comp Go-to-Landmark using Reinforcement Learning
#Implement DQN or DDPG as independent agents for collaborative action

import gym
import numpy as np
from gym import envs
import matplotlib.pyplot as plt
import seaborn as sns
import time
import make_env
sns.set()

env =make_env.make_env('simple_spread')
obs =env.reset()

print(obs)
print(env.observation_space, env.action_space, env.n)

env.render()
time.sleep(2)

gamma=0.95
alpha=0.3
count=1.0
n_ep=10000
r_tot=0.
#action space: [0]=NIL, [1]=R, [2]=L, [3]=U, [4]=D
action=[[1,0,0,0,0],[0,1,0,0,0],[0,0,1,0,0],[0,0,0,1,0],[0,0,0,0,1]]

#Implement DQN or DDPG as independent agents for collaborative action

for i in range(50):
	obs, reward, done, _ = env.step([action[1],action[3],action[4]])
	print(reward)
	env.render()
	time.sleep(0.5)

'''
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

def test_agent(env,q):
	prev_state=env.reset()
	print('Intelligent Agent Test')
	time.sleep(1.0)
	env.render()
	while True:
		action= np.argmax(q[prev_state])
		next_state, reward, done, info = env.step(action)
		next_action=np.argmax(q[next_state])
		prev_state=next_state
		action=next_action
		env.render()
		time.sleep(0.5)
		if done:
			break
c_re=[]
ep=[]
for i in range(n_ep):
	r=0
	epsilon=1/count
	prev_state=env.reset()
	print('::::::::::::::episode no.: ', i, '::::::::::::::')
	env.render()
	while True:

		action = ept_greed(prev_state,epsilon)
		next_state, reward, done, info = env.step(action)
		update_q(prev_state, action, reward,next_state)
		prev_state=next_state
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

test_agent(env,q)
time.sleep(0.5)
'''
'''
print('total reward: ', r_tot)
print('avg reward :', r_tot/n_ep)
print('final Q: ', q)
plt.plot(ep,c_re)
plt.title("Q-Learning on FrozenLake8x8-v0")
plt.ylim(0,1)
plt.xlabel("No. of iterations")
plt.ylabel("Cumulative Average Reward")
plt.show()
env.close()
'''
#end of code