#MABP python implement /w OpenAI Gym
#epsilon-greedy algorithm
import gym
import pandas as pd
import matplotlib.pyplot as plt
from gym import envs
import numpy as np
import gym_bandits

#create environment
env = gym.make('BanditTenArmedGaussian-v0')

#run this to rand_init the env
#number of actions
n_ban= 10

#expected reward table of actions, init to zeros
q_table=np.zeros(n_ban)

#tables to keep track of no. of times arm is pulled
n_table=np.zeros(n_ban)

#initialise ucb
ucb=np.zeros(n_ban)

#get initial observation
env.reset()

rew=[]
ep=[]
#get type and no. of actions
print(env.action_space)

#iterate over episodes/rounds
for episode_i in range(1000):
	print("\n\nEpisode number: ", episode_i)
	
	#ucb init
	if episode_i < 10:
		action = episode_i
	else:
		for i in range(n_ban):
			up_bound = np.sqrt((2*np.log(sum(n_table)))/n_table[i])
			ucb[i] = q_table[i]+up_bound
		action = np.argmax(ucb)

	print(pd.DataFrame({'UCB':ucb}))
	print("action taken:", action)

	#get return values on 4 variables after performing action
	observation, reward, done, info = env.step(action)

	#UPDATE
	n_table[action]+=1
	q_table[action]=q_table[action]+ (1/n_table[action])*(reward-q_table[action])
	rew.append(np.max(q_table))
	ep.append(episode_i)
	#See results
	print("Observation: ", observation)
	print("Reward: ", reward)
	show = pd.DataFrame({'Q-val':q_table, 'n_pulls': n_table})
	show.index.name='arm'
	print(show.head(10))

print("Most rewarding bandit is: ", np.argmax(q_table))
print("It gave a reward of: ", round(np.max(q_table),3))
plt.plot(ep,rew)
plt.show()

#close environment
env.close()

#end of code