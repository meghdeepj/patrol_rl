# RL Environment for Patrolling
__author__='meghdeep'

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
sns.set()
from itertools import compress
import os
import sys
import optparse
import time
import random

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary

sys.path.append(os.path.join('c:', os.sep, 'whatever', 'path', 'to', 'sumo', 'tools'))
sumoBinary = checkBinary("sumo-gui")
sumoCmd = [sumoBinary, "-c", "grid_asym.sumocfg", "--tripinfo-output", "tripinfo.xml"]

import traci

class rl_env(object):

    def __init__(self):
        self.nrow, self.ncol= 6, 6
        self.stateSpace=np.array([i for i in range(36)])
        self.actionSpace = [0, 1, 2, 3]
        #Action space= {'0': 'North', '1': 'South', '2': 'East', '3': 'West'}
        self.state=[0.0, 0.0, 0.0]
        self.nA = 4
        self.nS = 36
        self.reward=[0.0, 0.0, 0.0]

    def sample(self, nn):
        act_idx = random.choice([i for i in range(len(nn))])
        return act_idx

    def set_actionSpace(self, n_edges):
        self.actionSpace = n_edges

    def step(self, next_state, idle, i):
        self.state[i]=next_state
        # print('cur and next: ', curr_state, self.state)
        self.reward[i]=idle[self.state[i]]
        return self.state[i], self.reward

    def reward_out(self, idle, prev_node, i):
        self.reward[i] = idle[prev_node]
        return self.reward[i]

    def reset(self, route0, route1, route2):
        rou_curr=[[],[],[]]
        rou_curr[0].append(route0)
        rou_curr[1].append(route1)
        rou_curr[2].append(route2)
        self.state=[6, 35, 5]
        self.reward=[0.0, 0.0, 0.0]
        traci.start(sumoCmd)
        traci.route.add('rou_0', rou_curr[0])
        traci.route.add('rou_1', rou_curr[1])
        traci.route.add('rou_2', rou_curr[2])
        traci.vehicle.add(vehID = 'veh0',routeID = 'rou_0', typeID = "car1")
        traci.vehicle.add(vehID = 'veh1',routeID = 'rou_1', typeID = "car1")
        traci.vehicle.add(vehID = 'veh2',routeID = 'rou_2', typeID = "car1")
        return self.state
#end of class

def eval_met(idle, v_idle,sumo_step, n):

    avg_v_idl=np.zeros((36,1))
    max_v_idl=np.zeros((36,1))
    var_v_idl=np.zeros((36,1))
    #avg idleness
    for i in range(n):
        if v_idle[i]:
            avg_v_idl[i]=np.sum(np.square(v_idle[i]))/(2*sumo_step)
    # print(avg_v_idl)
    glo_v_idl=np.sum(avg_v_idl)/30
    glo_idl= np.sum(idle)/30
    #max idleness
    for i in range(n):
        if v_idle[i]:
            max_v_idl[i]=np.max(v_idle[i])
    glo_max_idl=np.max(idle)
    glo_max_v_idl=np.max(max_v_idl)
    #var,stdev and glob stdev
    for i in range(n):
        if v_idle[i]:
            var_v_idl[i]=(np.sum(np.power(v_idle[i],3))/(3*sumo_step))-avg_v_idl[i]
    sd_v_idl=np.sqrt(var_v_idl)
    glo_sd_v_idl=np.sum(sd_v_idl)/30
    return avg_v_idl, max_v_idl, sd_v_idl, glo_v_idl, glo_max_v_idl, glo_sd_v_idl, glo_idl, glo_max_idl
#end of fn

def negotiator(ps, cs, ns, nn, i):
    l=[]
    idx=[]
    n=[True]*len(nn)
    r=0
    l=[not(item) for item in [item in ns for item in nn]]
    idx= [j for j, x in enumerate(ps) if cs[i]==x]
    if len(idx) and len(nn):
        for i in idx:
            m=[not(item) for item in [item in [cs[i]] for item in nn]]
            n= [a and b for a, b in zip(n, m)]
    p= [a and b for a, b in zip(n, l)]
    an = list(compress(nn, p))
    if not an:
        p=[True]*len(nn)
        r=1
    return p,r
#end of fn

def BLRE_patrol(idle,c,env, an):

    neigh=[idle[i] for i in an]
    print(neigh)
    m = max(neigh)
    idx= [i for i, j in enumerate(neigh) if j == m]
    print('idx: ', idx)
    act_idx=random.choice(idx)
    action_node=an[act_idx]
    print('cur and next: ', c, action_node)
    if c==action_node:
            act_idx=CR_patrol(idle,c,env, an)
    return act_idx
#end of fn

def run(env):
    rou_curr0= "6to"+str(random.choice([12, 7]))
    rou_curr1= "35to"+str(random.choice([34,29]))
    rou_curr2= "5to"+str(random.choice([11,4]))
    # rou_curr0= "13to"+str(random.choice([18,12,14,8]))
    # rou_curr1= "11to"+str(random.choice([16,6,12,10]))
    rou_curr=[rou_curr0, rou_curr1, rou_curr2]
    env.reset(rou_curr0, rou_curr1, rou_curr2)
    sumo_step=1.0
    bn=[0, 17, 30, 31, 32, 33]
    cr=[0.0, 0.0, 0.0]
    rl_step=1.0
    idle=np.zeros((36,1))
    v_idle=[[] for _ in range(36)]
    edge=[0,0,0]
    prev_node=env.state
    curr_node=[0.0,0.0,0.0]
    temp_n=[0.,0.,0.]
    temp_p=[0,35,5]
    ga=[]
    gav=[]
    ss=[]
    while traci.simulation.getMinExpectedNumber()>0:

        traci.simulationStep()
        idle+=1
        for i in bn:
            idle[i]=0
        edge[0]=traci.vehicle.getRoadID('veh0')
        edge[1]=traci.vehicle.getRoadID('veh1')
        edge[2]=traci.vehicle.getRoadID('veh2')
        #print('veh edge data: ',edge)
        for i, ed in enumerate(edge):
            if ed and (ed[0]!=':'):
                curr_node[i]= ed.split('to')
                curr_node[i]=int(curr_node[i][1])
            elif ed[0]==':':
                curr_node[i]=ed[1:].split('_')
                curr_node[i]=int(curr_node[i][0])
        env.state=curr_node.copy()
        # print('p_node:',prev_node, 'temp_p: ', temp_p, 'c_node:',curr_node, 'temp_n: ', temp_n)
        # Action decision on new edge
        for i in range(3):
            if prev_node[i]!=curr_node[i]:
                temp_p[i]=prev_node[i]
                print(':::::::::::::to next node for', i, '::::::::::::::::')

                print('Veh angle: ', traci.vehicle.getAngle('veh'+str(i)))
                rou_step=[]
                prev_reward=env.reward_out(idle, prev_node[i], i)[0]
                print('reward on prev step: ', prev_reward)
                v_idle[int(prev_node[i])].append(prev_reward.copy())
                avg_v_idl, max_v_idl, sd_v_idl, glo_v_idl, glo_max_v_idl, glo_sd_v_idl, glo_idl, glo_max_idl = eval_met(idle, v_idle,sumo_step, 36)
                print('global avg node visit idleness: ', glo_v_idl, '\nglobal max node visit idleness: ', glo_max_v_idl)
                print('global avg instant idleness: ', glo_idl, '\nglobal max instant idleness: ', glo_max_idl)
                gav.append(glo_v_idl)
                ga.append(glo_idl)
                ss.append(sumo_step)
                cr[i]+=prev_reward
                idle[int(prev_node[i])]=0
                print(idle.reshape(6,6))

                #Use links to decide actions
                links = traci.lane.getLinks(traci.vehicle.getLaneID('veh'+str(i)), extended=False)
                s_lanes = [i[0] for i in links]
                n_edges=[]
                n_nodes=[]
                for nodes in s_lanes:
                    n_edges.append(nodes.split('_')[0])
                for edges in n_edges:
                    n_nodes.append(int(edges.split('to')[1]))
                print(n_edges, n_nodes)
                env.set_actionSpace(n_edges)
                p, r=negotiator(temp_p, curr_node, temp_n, n_nodes, i)
                an = list(compress(n_nodes, p))
                ae = list(compress(n_edges, p))
                print('allowed nodes{}'.format(an))
                if r==1:
                    act_idx=env.sample(an)
                else:
                    act_idx=BLRE_patrol(idle,curr_node[i],env, an)
                action = ae[act_idx]
                next_state, reward = env.step(an[act_idx], idle, i)
                temp_n[i]=next_state
                print('action: ', action, 'next_state: ', next_state, 'reward: ', reward)
                #print('curr_node after step: ',curr_node, env.state)
                rou_new=action
                rou_step.append(rou_curr[i])
                rou_step.append(rou_new)
                print('next_route: ', rou_step)
                traci.vehicle.setRoute(vehID = 'veh'+str(i), edgeList = rou_step)
                rou_curr[i]=rou_new

        prev_node=curr_node.copy()
        #print('curr route: ',rou_curr)
        sumo_step+=1
        if sumo_step ==20000:
            break

    plt.plot(ss,ga, "-r", linewidth=0.6,label="Global Average Idleness")
    plt.plot(ss,gav, "-b", linewidth=4, label="Global Average Node Visit Idleness")
    plt.legend(loc="lower right")
    up=np.ceil(max(ga)/10)*10
    plt.yticks(np.linspace(0,up,(up/10)+1, endpoint=True))
    plt.xlabel('Unit Time')
    plt.ylabel('Idleness')
    plt.title('Performance')
    traci.close()
    plt.show()
    sys.stdout.flush()
#end of fn

if __name__ == '__main__':
    env=rl_env()
    run(env)
#end of main

#end of code
