#MDP-DP python implement /w OpenAI Gym
#value-iteration algorithm
from IPython.core.debugger import set_trace
import numpy as np
import gym

from src.rlenvs.rlenvs.envs.gridworld import GridworldEnv
#env=gym.make('FrozenLake-v0')

env=GridworldEnv()
def value_iter(policy, V,env,gamma,theta):
	while True:
		delta=0
		for s in range(env.nS):
			A=next_value(s,V,gamma)
			best=np.max(A)
			delta=max(delta,np.abs(best-V[s]))
			V[s]=best
		if delta<theta:
			break
		print(V.reshape(4,4))
	for s in range(env.nS):
		A=next_value(s,V,gamma)
		policy[s]=np.argmax(A)
	return V,policy	

def next_value(state,V,gamma):
	A=np.zeros(env.nA)
	for a in range(env.nA):
		for prob,next_state,reward,done in env.P[state][a]:
			A[a]+=prob*(reward+gamma*V[next_state])
	return A

policy_init = np.zeros([env.nS, env.nA])/env.nA
V_init=np.zeros(env.nS)
print(policy_init,'\n' ,V_init.reshape(4,4))

best_V, best_pol =value_iter(policy_init, V_init, env, 1.0, 0.00001)

print('\n',best_V.reshape(4,4),'\n',best_pol[:,0].reshape(4,4))