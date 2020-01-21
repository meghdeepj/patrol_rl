#!/usr/bin/env python

'''
Online module of TPBP Algo on walks
'''
import rospy
import rospkg
import os, sys
import networkx as nx
import ConfigParser as CP
from mrpp_algos.msg import TaskDone, NextTask
from sets import Set
import numpy

def tpbp_reward(g, algo_params, walk, priority_nodes, time_periods, assigned):
    nodes = list(Set(walk))
    term1 = 0.
    term2 = 0.
    for i in nodes:
        term1 += g.node[i]['idleness']
        if i in priority_nodes:
            j = priority_nodes.index(i)
            # print "here2", j
            if not assigned[j]:
                term2 += max(g.node[i]['idleness'] - time_periods[j], 0)
    term3 = 0.
    for i in range(len(priority_nodes)):
        if not assigned[i] and not priority_nodes[i] in nodes:
            dist = numpy.inf
            for j in nodes:
                temp = nx.shortest_path_length(g, j, priority_nodes[i])
                if temp < dist:
                    dist = temp
            term3 += dist
    term4 = 0.
    for i in range(len(walk) - 1):
        term4 += g[walk[i]][walk[i + 1]]['length']
    return numpy.dot([term1, term2, term3, term4], algo_params)

def tpbp_walk(g, algo_params, cur_node, priority_nodes, time_periods, assigned, folder_path):
    best_reward = -numpy.inf
    best_walk = []
    for j in range(len(priority_nodes)):
        if not assigned[j]:
            #print ('/valid_trails_{}_{}_{}.in'.format(cur_node, priority_nodes[j], str(int(time_periods[priority_nodes.index(cur_node)]))))
            with open(folder_path + '/valid_trails_{}_{}_{}.in'.format(cur_node, priority_nodes[j], str(int(time_periods[priority_nodes.index(cur_node)]))), 'r') as f:
                count = 0
                for line in f:
                    count += 1
                    line1 = line.split('\n')
                    line2 = line1[0].split(' ')
                    r = tpbp_reward(g, algo_params, line2, priority_nodes, time_periods, assigned)
                    if r > best_reward:
                        best_reward = r
                        best_walk = line2
    return best_walk

class TPBP:

    def __init__(self, sim_set):
        rospy.init_node('tpbp_online', anonymous = True)
        rospy.Subscriber('/task_done', TaskDone, self.callback)
        rate = rospy.Rate(10) # 10hz
        self.dirname = rospkg.RosPack().get_path('mrpp_algos')
        self.config_file = self.dirname + '/config.txt'
        self.sim_set = sim_set
        config = CP.ConfigParser()
        config.read(self.config_file)
        params = {}
        for option in config.options(sim_set):
            params[option] = config.get(sim_set, option)
        if params['algo'] != 'tpbp_walk':
            return
        self.dest_folder = self.dirname + '/' + params['folder']
        self.num_robots = int(params['num_robots'])
        # print self.num_robots
        self.priority_nodes = params['priority_nodes'].split(' ')
        self.time_periods = map(float, params['time_periods'].split(' '))
        self.graph = nx.read_graphml(self.dirname + '/graph_ml/' + params['graph'] + '.graphml')
        for node in self.graph.nodes():
            self.graph.node[node]['idleness'] = 0.
        self.algo_params = map(float, params['algo_params'].split(' '))
        self.stamp = 0.0
        self.assigned = []
        for i in self.priority_nodes:
            self.assigned.append(False)
        self.robot_cur_walks = {}
        self.robot_pub = []
        task_list = []

        for i in range(self.num_robots):
            self.robot_cur_walks[i] = tpbp_walk(self.graph, self.algo_params, self.priority_nodes[i], self.priority_nodes, self.time_periods, self.assigned, self.dest_folder)
            self.assigned[self.priority_nodes.index(self.robot_cur_walks[i][-1])] = True
            task_list.append(map(str, self.robot_cur_walks[i]))


        for i in range(self.num_robots):
            #print i
            self.robot_pub.append(rospy.Publisher('/robot_{}/next_task'.format(i), NextTask, queue_size = 10))
            rospy.sleep(1.)
            pubs = NextTask()
            pubs.task = task_list[i]
            #print pubs.task
            self.robot_pub[i].publish(pubs)


    def callback(self, data):
        #print 'here', data.stamp, data.robot_id, data.node_id
        if self.stamp < data.stamp:
            #Update Idleness
            dev = data.stamp - self.stamp
            self.stamp = data.stamp
            for i in self.graph.nodes():
                self.graph.node[i]['idleness'] += dev
            for i in range(len(data.node_id)):
                    self.graph.node[str(data.node_id[i])]['idleness'] = 0

            #Update Walk If Necessary
            for i in range(len(data.robot_id)):
                #print data.robot_id[i], self.robot_cur_walks[data.robot_id[i]]
                # for j in self.graph.nodes():
                #     # print j, self.graph.node[j]['idleness']
                n = data.node_id[i]
                m = self.robot_cur_walks[data.robot_id[i]][0]
                # print str(n), m
                if n == m:
                    self.robot_cur_walks[data.robot_id[i]].pop(0)
                    print data.robot_id[i], self.robot_cur_walks[data.robot_id[i]]
                if len(self.robot_cur_walks[data.robot_id[i]]) == 0:
                    self.assigned[self.priority_nodes.index(str(n))] = False
                    self.robot_cur_walks[data.robot_id[i]] = tpbp_walk(self.graph, self.algo_params, str(n), self.priority_nodes, self.time_periods, self.assigned, self.dest_folder)
                    self.assigned[self.priority_nodes.index(self.robot_cur_walks[data.robot_id[i]][-1])] = True
                    pubs = NextTask()
                    pubs.task = map(str, self.robot_cur_walks[data.robot_id[i]])
                    self.robot_pub[data.robot_id[i]].publish(pubs)


if __name__ == '__main__':
    if len(sys.argv[1:]) >= 1:
        t = TPBP(sys.argv[1])
        done = False
        while not done:
            done = rospy.get_param('/done')
    else:
        print 'Please pass the appropriate arguments'
