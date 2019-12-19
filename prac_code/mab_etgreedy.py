#MABP python implement /w OpenAI Gym
#epsilon-t-greedy algorithm
import gym
import matplotlib.pyplot as plt
import pandas as pd
from gym import envs
import numpy as np
import gym_bandits

#create environment
env = gym.make('BanditTenArmedGaussian-v0')

#run this to rand_init the env
env.seed(34)

#number of actions
n_ban= 10

#expected reward table of actions, init to zeros
q_table=np.zeros(n_ban)

#tables to keep track of no. of times arm is pulled
n_table=np.zeros(n_ban)

#get initial observation
env.reset()
count = 1.0

#get type and no. of actions
print(env.action_space)

#visualise convergence
rew=[]
ep =[]

#iterate over episodes/rounds
for instance in range(100):
	print("\n\nEpisode number: ", instance)
	
	r = np.random.random(1)[0]
	print("random number: ", r)
	
	#decrese epsilon
	epsilon = 1/count

	print('Epsilon: ', epsilon)
	#make the agent
	if r >= epsilon:
		best_idx= np.argmax(q_table)
		action=best_idx
	else:
		action=env.action_space.sample()

	print("action taken:", action)

	#get return values on 4 variables after performing action
	observation, reward, done, info = env.step(action)
	rew.append(np.max(q_table))
	ep.append(instance)
	
	UPDATE
	n_table[action]+=1
	q_table[action]=q_table[action]+ (1/n_table[action])*(reward-q_table[action])
	count+=0.2
		
	#See updates
	print("Observation: ", observation)
	print("Reward: ", reward)
	show = pd.DataFrame({'Q-val':q_table, 'n_pulls': n_table})
	show.index.name='arm'
	print(show.head(10))

#results
print("Most rewarding bandit is: ", np.argmax(q_table))
print("It gave a reward of: ", round(np.max(q_table),3))
plt.plot(ep,rew)
plt.show()

#close environment
env.close()

#end of code