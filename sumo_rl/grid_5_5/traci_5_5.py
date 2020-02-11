#!/usr/bin/env python

import numpy, random, time
# from mrpp_algos.msg import NextTask, TaskDone

import os, sys, optparse
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

def run():
    step=0
    while traci.simulation.getMinExpectedNumber()>0:
        traci.simulationStep()
        #print(step)
        if (not step%3):
            edge=traci.vehicle.getRoadID('veh0')
            if edge and (edge[0]!=':'):
                print(edge)
                node= edge.split('to')
                print(node[1])
        if (traci.vehicle.getRoadID('veh0')=='1to0'):
            traci.vehicle.setRoute('veh0',  ['1to0', '0to5', '5to6', '6to7', '7to2', '2to1', '1to0'])
        step+=1
    traci.close()
    sys.stdout.flush()
#end of fn

if __name__ == '__main__':
    traci.start(sumoCmd)
    run()
#end of fn

#end of code
