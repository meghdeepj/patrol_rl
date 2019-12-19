#MABP python implement /w OpenAI Gym
#epsilon-greedy algorithm
import gym
import pandas as pd
from gym import envs
import numpy as np
import gym_bandits

#create environment
env = gym.make('BanditTenArmedGaussian-v0')

#run this to rand_init the env
np.random.seed(42)
env.seed(34)

#number of actions
n_ban= 10

#expected reward table of actions, init to zeros
q_table=np.zeros(n_ban)

#tables to keep track of no. of times arm is pulled
n_table=np.ones(n_ban)

#epsilon value
epsilon=0.2

#get initial observation
observation = env.reset()
print("Init Observation: ", observation)
#get type and no. of actions
print(env.action_space)

#iterate over episodes/rounds
for episode_i in range(20):
	print("\n\nEpisode number: ", episode_i)
	
	r = np.random.random(1)[0]
	print("random number: ", r)
	
	#make the agent
	if r >= epsilon:
		best_idx= np.argmax(q_table)
		action=best_idx
	else:
		action=env.action_space.sample()

	print("action taken:", action)

	#get return values on 4 variables after performing action
	observation, reward, done, info = env.step(action)

	#UPDATE
	q_table[action]=q_table[action]+ (1/n_table[action])*(reward-q_table[action])
	n_table[action]+=1
	
	#See results
	print("Observation: ", observation)
	print("Reward: ", reward)
	show = pd.DataFrame({'Q-val':q_table, 'n_pulls': n_table})
	show.index.name='arm'
	print(show.head(10))

print("Most rewarding bandit is: ", np.argmax(q_table))
print("It gave a reward of: ", round(np.max(q_table),3))

#close environment
env.close()

#end of code