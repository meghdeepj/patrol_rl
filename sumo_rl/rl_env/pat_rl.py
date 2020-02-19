# RL Environment for Patrolling
__author__='meghdeep'

import numpy as np
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
sumoCmd = [sumoBinary, "-c", "grid_5_5.sumocfg", "--tripinfo-output", "tripinfo.xml"]

import traci

class rl_env(object):

    def __init__(self):
        self.nrow, self.ncol= 5, 5
        self.stateSpace=np.array([i for i in range(25)])
        self.actionSpace = [0, 1, 2, 3]
        #Action space= {'0': 'North', '1': 'South', '2': 'East', '3': 'West'}
        self.state=0
        self.nA = 4
        self.nS = 25
        self.reward=0

    def sample(self):
        action = random.choice(self.actionSpace)
        return action

    def check_step(self, curr_state, next_state, idle, action):
        if curr_state==next_state:
            action=self.sample()
            self.state, self.reward, action=self.step(action, idle)
        return self.state, self.reward, action

    def step(self, action, idle):
        curr_state=self.state
        row=curr_state//5
        col=curr_state%5
        if action == 3:
            col = max(col-1, 0)
        elif action == 0:
            row = min(row+1,self.nrow-1)
        elif action == 2:
            col = min(col+1,self.ncol-1)
        elif action == 1:
            row = max(row-1,0)
        self.state=row*5+col
        # print('cur and next: ', curr_state, self.state)
        self.reward=idle[self.state]
        self.state, self.reward, action=self.check_step(curr_state, self.state, idle, action)
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
    avg_v_idl=np.zeros((25,1))
    max_v_idl=np.zeros((25,1))
    var_v_idl=np.zeros((25,1))
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

def CR_patrol(idle, c, env):

    row=c//5
    col=c%5
    neigh=[idle[min(row+1,env.nrow-1)*5+col], idle[max(row-1,0)*5+col], idle[row*5+min(col+1,env.ncol-1)], idle[row*5+max(col-1, 0)]]
    if c==0:
        neigh[1], neigh[3]=0,0
    elif c==20:
        neigh[0], neigh[3]=0,0
    elif c==24:
        neigh[0], neigh[2]=0,0
    elif c==4:
        neigh[1], neigh[2]=0,0
    elif c==21 or c==22 or c==23:
        neigh[0]=0
    elif c==1 or c==2 or c==3:
        neigh[1]=0
    elif c==9 or c==14 or c==19:
        neigh[2]=0
    elif c==5 or c==10 or c==15:
        neigh[3]=0
    print(neigh)
    m = max(neigh)
    idx= [i for i, j in enumerate(neigh) if j == m]
    print('idx: ', idx)
    action=random.choice(idx)
    if action == 3:
        col = max(col-1, 0)
    elif action == 0:
        row = min(row+1,env.nrow-1)
    elif action == 2:
        col = min(col+1,env.ncol-1)
    elif action == 1:
        row = max(row-1,0)
    n=row*5+col
    print('cur and next: ', c, n)
    if c==n:
            action=CR_patrol(idle,c,env)
    return action

#end of fn

def run(env):

    rou_curr= "0to"+str(random.choice([1,5]))
    env.reset(rou_curr)
    sumo_step=1.0
    cr=0.0
    rl_step=1.0
    idle=np.zeros((25,1))
    v_idle=[[] for _ in range(25)]
    edge=[0,0]
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

            avg_v_idl, max_v_idl, sd_v_idl, glo_v_idl, glo_max_v_idl, glo_sd_v_idl, glo_idl, glo_max_idl = eval_met(idle, v_idle,sumo_step, 25)
            print('global avg node visit idleness: ', glo_v_idl, '\nglobal max node visit idleness: ', glo_max_v_idl)
            print('global avg instant idleness: ', glo_idl, '\nglobal max instant idleness: ', glo_max_idl)
            #print(np.array(v_idle).reshape(5,5))
            rl_step+=1
            cr+=prev_reward
            acr=cr/sumo_step
            print('acr: ', acr)
            print('')
            idle[int(prev_node)]=0
            print(idle.reshape(5,5))
            #action=env.sample()
            action=CR_patrol(idle,curr_node,env)
            #action=q_patrol(idle,curr_node,env)
            next_state, reward, action = env.step(action, idle)
            print('action: ', action, 'next_state: ', next_state, 'reward: ', reward)
            rou_new=str(curr_node)+'to'+str(next_state)
            rou_step.append(rou_curr)
            rou_step.append(rou_new)
            print('next_route: ', rou_step)
            print(':::::::::::::to next node::::::::::::::::')
            traci.vehicle.setRoute(vehID = 'veh0', edgeList = rou_step)
            rou_curr=rou_new

        prev_node=curr_node
        sumo_step+=1

    traci.close()
    sys.stdout.flush()
#end of fn

if __name__ == '__main__':
    #traci.start(sumoCmd)
    # route_0 = ["0to5", "5to10", "10to15", "15to20", "20to21", "21to16", "16to11", "11to6", "6to1", "1to0"]
    #traci.route.add('rou_0', route_0)
    #traci.vehicle.add(vehID = 'veh0',routeID = 'rou_0', typeID = "car1")
    #traci.vehicle.setStop(vehID = 'veh0', edgeID = '1to0', duration = 2000.)
    env=rl_env()
    run(env)
#end of main

#end of code
