#OpenAI Gym basics
#Create Environment for cart-pole 

import gym
import time
env = gym.make('CartPole-v0')
env.reset()

for i in range(1000):
	env.render()
	env.step(env.action_space.sample())
	time.sleep(.02)
env.close()

#end of code