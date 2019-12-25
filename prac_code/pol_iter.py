#MDP-DP python implement /w OpenAI Gym
#policy-iteration algorithm
from IPython.core.debugger import set_trace
import numpy as np
import gym

from src.rlenvs.rlenvs.envs.gridworld import GridworldEnv
#env=gym.make('FrozenLake-v0')

env=GridworldEnv()

print(env.nS, env.nA)

def policy_eval(policy,env,V,gamma=1.0,theta=0.00001):
	V=np.zeros(env.nS)
	while True:
		delta=0
		for s in range(env.nS):
			v=0
			for a in range(env.nA):
				for prob, next_state, reward, done in env.P[s][a]:
					v+=prob*(reward+gamma*V[next_state])
			delta=max(delta,np.abs(v-V[s]))
			V[s]=v
		if delta<theta:
			return np.array(V)


def eval_all_states(policy, env, V, gamma,delta):
	#V_new=np.zeros(env.nS)
	for s in range(env.nS):
		V_new=cal_state_value(policy,s,env,V,gamma)
		delta=max(delta,np.abs(V_new-V[s]))
		V[s]=V_new
	return V, delta

def cal_state_value(policy, state, env, V, gamma):
	v=0
	for a in range(env.nA):
		for prob, next_state, reward, done in env.P[state][a]:
			v+=prob*(reward+gamma*V[next_state])
	return v

def policy_iter(policy,env,gamma):
	pol_stable=True
	c=0
	while c<10:
		V=policy_eval(policy,env,gamma,0.0001)
		print(V.reshape(4,4))
		policy,pol_stable=iter_all_states(policy,env,V,gamma,pol_stable)
		#print(policy)
		if pol_stable:
			break
		c+=1
	return np.array(V),policy	
		

def iter_all_states(policy,env,V,gamma,pol_stable):
	for s in range(env.nS):
		pi=policy[s]
		policy[s]=cal_max_policy(policy,s,env,V,gamma)
	if(np.all(pi==policy)):
		print('policy converged:')
		pol_stable=True
	else:
		print('policy not-converged:')
		pol_stable=False
	return policy, pol_stable

def cal_max_policy(policy,state,env,V,gamma):
	Q=np.zeros((env.nA,1))
	for a ,action_prob in enumerate(policy[state]):
		for prob, next_state, reward, done in env.P[state][a]:
			Q[a]+=prob*(reward+gamma*V[next_state])
	policy[state]=np.argmax(Q)
	return policy[state]

policy_init = np.zeros([env.nS, env.nA])/env.nA
print(policy_init)

best_V, best_pol =policy_iter(policy_init,env,1.0)

print(best_V.reshape(4,4),best_pol[:,1].reshape(4,4))

#end of code