#!/usr/bin/env python

import rospy
import rospkg
import os, sys
import ConfigParser as CP
from mrpp_algos.msg import TaskDone
import networkx as nx
import pandas as pd


class Record_Data:

    def __init__(self, sim_set):
        rospy.init_node('record_data', anonymous = True)
        self.dirname = rospkg.RosPack().get_path('mrpp_algos')
        self.config_file = self.dirname + '/config.txt'
        self.sim_set = sim_set
        config = CP.ConfigParser()
        config.read(self.config_file)
        params = {}
        for option in config.options(sim_set):
            params[option] = config.get(sim_set, option)
        self.dest_file = self.dirname + '/' + params['folder'] + '/sim_data.in'
        self.dest_folder = self.dirname + '/' + params['folder']
        rospy.Subscriber('/task_done', TaskDone, self.callback)
        self.stamp = -1.
        self.graph_name = config.get(self.sim_set, 'graph')
        self.num_robots = int(config.get(self.sim_set, 'num_robots'))
        self.graph = nx.read_graphml(self.dirname + '/graph_ml/' + self.graph_name + '.graphml')
        self.total_time  = int(config.get(self.sim_set, 'min_time'))

    def callback(self, data):
        if self.stamp < data.stamp:
            self.stamp = data.stamp
            with open(self.dest_file, 'a+') as f:
                f.write(str(data.stamp) + '\n')
                f.write(' '.join(map(str, data.node_id)) + '\n')
                f.write(' '.join(map(str, data.robot_id)) + '\n')

    def post_process(self):
        count = 0        
        nodes = self.graph.nodes()
        node_data = {}
        for i in nodes:
            node_data[i] = []        
        time_stamps = []
        robot_data = {}
        for i in range(self.num_robots):
            robot_data[i] = [] 
        # node_data = pd.DataFrame()
        nod = []
        rob = []
        time = 0
        with open(self.dest_file) as f:
            for line in f:
                if count % 3 == 0:
                    time = int(float(line.strip('\n')))
                    time_stamps.append(time)
                elif count % 3 == 1:
                    l = line.strip('\n')
                    nod = l.split(' ')
                    for i in nod:
                        node_data[i].append(time)
                else:
                    l = line.strip('\n')
                    rob = map(int, l.split(' '))
                    for i in range(len(rob)):
                        robot_data[rob[i]].append((time, nod[i]))
                count += 1

        for i in range(self.num_robots):
            rob_file = self.dest_folder + '/robot_{}.in'.format(i)
            df = pd.DataFrame(robot_data[i])
            df.to_csv(rob_file, sep = ' ', index = False, header = False)
        
        node_data2 = {}
        for i in nodes:
            node_data2[i] = [0]
            for j in range(1, self.total_time + 1):
                if j in node_data[i]:
                    node_data2[i].append(0)
                else:
                    node_data2[i].append(node_data2[i][-1] + 1)
        
        nod_file = self.dest_folder + '/node_idleness.csv'
        df = pd.DataFrame(node_data2)
        df.to_csv(nod_file)



if __name__ == '__main__':
    if len(sys.argv[1:]) >= 1:
        t = Record_Data(sys.argv[1])
        done = False
        while not done:
            done = rospy.get_param('/done')
        t.post_process()
    else:
        print 'Please pass the appropriate arguments'
