# RL Environment for Patrolling
__author__='meghdeep'

import numpy as np, os, sys, optparse
from random import random
from time import time

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
        self.stateSpace=[i for i in range(25)]
        self.actionSpace = {'N': 5, 'S': -5, 'E':1, 'W':-1}
        
    def state(self):
        pass

    def step(self, action):
        pass

    def reward(self):
        pass
#end of class

def run():
    step=0
    idle=np.zeros((25,1))
    prev_node=0
    curr_node=0
    while traci.simulation.getMinExpectedNumber()>0:
        traci.simulationStep()
        edge=traci.vehicle.getRoadID('veh0')
        if True:
            print(edge)
            prev_node=curr_node
            if edge and (edge[0]!=':'):
                curr_node= edge.split('to')
                curr_node=curr_node[1]
                print('p_node:',prev_node, 'c_node:',curr_node)
            elif edge[0]==':':
                curr_node=edge[1:].split('_')
                curr_node=curr_node[0]
                print('p_node:',prev_node, 'c_node:',curr_node)
        if (traci.vehicle.getRoadID('veh0')=='1to0'):
            traci.vehicle.setRoute('veh0',  ['1to0', '0to5', '5to6', '6to7', '7to2', '2to1', '1to0'])
        step+=1
        idle+=1
        if prev_node!=curr_node:
            idle[int(prev_node)]=0
        print(idle.reshape(5,5))
    traci.close()
    sys.stdout.flush()
#end of fn

if __name__ == '__main__':
    traci.start(sumoCmd)
    route_0 = ["0to5", "5to10", "10to15", "15to20", "20to21", "21to16", "16to11", "11to6", "6to1", "1to0"]
    traci.route.add('rou_0', route_0)
    traci.vehicle.add(vehID = 'veh0',routeID = 'rou_0', typeID = "car1")
    run()
#end of main

#end of code
