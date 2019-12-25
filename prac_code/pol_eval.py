#MDP-DP python implement /w OpenAI Gym
#policy-evaluation algorithm
from IPython.core.debugger import set_trace
import numpy as np
from src.rlenvs.rlenvs.envs.gridworld import GridworldEnv

env=GridworldEnv()

print(env.nS, env.nA)


def policy_eval(policy,env,gamma=1.0,theta=0.00001):
	V=np.zeros(env.nS)
	while True:
		V,delta=over_all_states(policy,env,V,gamma,0)
		if delta<theta:
			return np.array(V)


def over_all_states(policy, env, V, gamma,delta):
	#V_new=np.zeros(env.nS)
	for s in range(env.nS):
		v=V[s]
		V[s]=cal_state_value(policy,s,env,V,gamma)
		delta=max(delta,np.abs(v-V[s]))
	return V, delta

def cal_state_value(policy, state, env, V, gamma):
	v=0
	for a ,action_prob in enumerate(policy[state]):
		for prob, next_state, reward, done in env.P[state][a]:
			v+=action_prob*prob*(reward+gamma*V[next_state])
	return v


policy = np.ones([env.nS, env.nA])/env.nA
print(policy)

V = policy_eval(policy,env,0.95,0.00001)

print(V.reshape(4,4))
#end of code