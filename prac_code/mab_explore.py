#MABP python implement /w OpenAI Gym
#pure exploratory
import gym
from gym import envs
import numpy as np
import gym_bandits

np.random.seed(42)
#print(envs.registry.all())

env = gym.make('BanditTenArmedGaussian-v0')

observation = env.reset()
print(env.action_space)

for episode_i in range(6):
	print("Episode number: ", episode_i)
	action= env.action_space.sample()
	print("action taken:", action)

	observation, reward, done, info = env.step(action)

	print("Observation: ", observation)
	print("Reward: ", reward)
	#print("Observation: ", observation)
	#print("Observation: ", observation)
env.close()



#end of code