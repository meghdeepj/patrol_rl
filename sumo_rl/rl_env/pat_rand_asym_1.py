# RL Environment for Patrolling
__author__='meghdeep'

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
sns.set()
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
# sumoCmd = [sumoBinary, "-c", "grid_5_5.sumocfg", "--tripinfo-output", "tripinfo.xml"]
sumoCmd = [sumoBinary, "-c", "grid_asym.sumocfg", "--tripinfo-output", "tripinfo.xml"]

import traci

class rl_env(object):

    def __init__(self):
        self.nrow, self.ncol= 6, 6
        self.stateSpace=np.array([i for i in range(36)])
        self.actionSpace = [0, 1, 2, 3]
        #Action space= {'0': 'North', '1': 'South', '2': 'East', '3': 'West'}
        self.state=0
        self.nA = 4
        self.nS = 36
        self.reward=0

    def sample(self):
        action = random.choice(self.actionSpace)
        return action

    def set_actionSpace(self, n_edges):
        self.actionSpace = n_edges

    def step(self, action, idle):
        curr_state=self.state
        row=curr_state//6
        col=curr_state%6
        if action == 3:
            col = max(col-1, 0)
        elif action == 0:
            row = min(row+1,self.nrow-1)
        elif action == 2:
            col = min(col+1,self.ncol-1)
        elif action == 1:
            row = max(row-1,0)
        self.state=row*6+col
        # print('cur and next: ', curr_state, self.state)
        self.reward=idle[self.state]
        #self.state, self.reward, action=self.check_step(curr_state, self.state, idle, action)
        return self.state, self.reward, action

    def reward_out(self, idle, prev_node):
        self.reward = idle[prev_node]
        return self.reward

    def reset(self, route0):
        rou_curr=[]
        rou_curr.append(route0)
        self.state=0
        self.reward=0
        traci.start(sumoCmd)
        traci.route.add('rou_0', rou_curr)
        traci.vehicle.add(vehID = 'veh0',routeID = 'rou_0', typeID = "car1")
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
    glo_v_idl=np.mean(avg_v_idl)
    glo_idl= np.mean(idle)
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
    glo_sd_v_idl=np.mean(sd_v_idl)
    return avg_v_idl, max_v_idl, sd_v_idl, glo_v_idl, glo_max_v_idl, glo_sd_v_idl, glo_idl, glo_max_idl
#end of fn

def run(env):

    rou_curr= "12to"+str(random.choice([13]))
    env.reset(rou_curr)
    sumo_step=1.0
    cr=0.0
    rl_step=1.0
    idle=np.zeros((36,1))
    v_idle=[[] for _ in range(36)]
    edge=[0]
    ga=[]
    gav=[]
    ss=[]
    prev_node=env.state
    while traci.simulation.getMinExpectedNumber()>0:

        traci.simulationStep()
        idle+=1
        edge=traci.vehicle.getRoadID('veh0')
        if edge and (edge[0]!=':'):
            curr_node= edge.split('to')
            curr_node=int(curr_node[1])
        elif edge[0]==':':
            curr_node=edge[1:].split('_')
            curr_node=int(curr_node[0])
        env.state=curr_node


        # Action decision on new edge
        if prev_node!=curr_node:
            print(edge)
            print('p_node:',prev_node, 'c_node:',curr_node)
            print('Veh angle: ', traci.vehicle.getAngle('veh0'))
            rou_step=[]
            prev_reward=env.reward_out(idle, prev_node)[0]
            print('reward on prev step: ', prev_reward)
            v_idle[int(prev_node)].append(prev_reward.copy())

            avg_v_idl, max_v_idl, sd_v_idl, glo_v_idl, glo_max_v_idl, glo_sd_v_idl, glo_idl, glo_max_idl = eval_met(idle, v_idle,sumo_step, 36)
            print('global avg node visit idleness: ', glo_v_idl, '\nglobal max node visit idleness: ', glo_max_v_idl)
            print('global avg instant idleness: ', glo_idl, '\nglobal max instant idleness: ', glo_max_idl)
            gav.append(glo_v_idl)
            ga.append(glo_idl)
            ss.append(sumo_step)
            cr+=prev_reward
            acr=cr/sumo_step
            idle[int(prev_node)]=0
            print(idle.reshape(6,6))
            lane = traci.vehicle.getLaneID('veh0')
            links = traci.lane.getLinks(lane, extended=False)
            s_lanes = [i[0] for i in links]
            n_edges=[]
            n_nodes=[]
            for nodes in s_lanes:
                n_edges.append(nodes.split('_')[0])
            for edges in n_edges:
                n_nodes.append(int(edges.split('to')[1]))
            print(n_edges, n_nodes)
            env.set_actionSpace(n_edges)
            action=env.sample()
            # next_state, reward, action = env.step(action, idle)
            #print('action: ', action, 'next_state: ', next_state, 'reward: ', reward)
            rou_step.append(rou_curr)
            rou_step.append(action)
            print('next_route: ', rou_step)
            print(':::::::::::::to next node::::::::::::::::')
            traci.vehicle.setRoute(vehID = 'veh0', edgeList = rou_step)
            rou_curr=action

        prev_node=curr_node
        sumo_step+=1
        if sumo_step ==20000:
            break
    plt.plot(ss,ga, "-r", linewidth=0.6,label="Global Average Idleness")
    plt.plot(ss,gav, "-b", linewidth=4, label="Global Average Node Visit Idleness")
    plt.legend(loc="lower right")
    up=np.ceil(max(ga)/20)*20
    plt.yticks(np.linspace(0,up,(up/20)+1, endpoint=True))
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
