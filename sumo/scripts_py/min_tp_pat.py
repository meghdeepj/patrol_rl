#!/usr/bin/env python

# from mrpp_ideal.msg import VisitNode
import graphs as gh

import networkx as nx
import random as rn
# import rospy
import sys
import rospkg
import ConfigParser as CP
import numpy as np
import os
# from std_msgs.msg import Int32

def min_tp_not_all_nodes(graph, noi):
    h = gh.multi_root_forest_arborescence(graph, noi)
    min_tp = np.inf

    for i in noi:
        for j in graph.nodes():
            if j not in noi and nx.has_path(h, i, j):
                l = nx.algorithms.shortest_path_length(h, i, j)
                if l < min_tp:
                    min_tp = l
    min_tp *= 2.

    len_cp = 0.
    f = gh.nodes_complete_subgraph(graph, noi)
    for i in f.edges():
        if f.edge[i[0]][i[1]]['length'] > len_cp:
            len_cp = f.edge[i[0]][i[1]]['length']
    return max(min_tp, len_cp)

def sterling_number_of_the_second_kind (n, k):
    # #Recursive
    # if k > n :
    #     return 0
    # if n == k:
    #     return 1
    # if k == 1:
    #     return 1
    # else:
    #     return k * sterling_number_of_the_second_kind(n - 1, k) + sterling_number_of_the_second_kind(n - 1, k - 1)
    
    #Iterative
    last_vals = [0] * k
    for i in range(1, n + 1):
        temp_vals = last_vals[:]
        for j in range(k):
            if j == 0:
                last_vals[j] = 1
            elif j == (i - 1):
                last_vals[j] = 1
            elif j > (i - 1):
                last_vals[j] = 0
            else:
                last_vals[j] = (j + 1) * temp_vals[j] + temp_vals[j - 1]
    return last_vals[k - 1]

def all_partitions_of_s(s, k, folder):
    #Input: 1. A set to be partitioned 2. Max number of partitions (Number of agents) 3. Folders to store the files generated
    #Output: Files indexed by number of partitions

    with open(folder + '/part_{}_{}.in'.format(0, 0), 'w') as f:
        f.write(str(s[0]) + '\n')

    for i in range(1, len(s)):
        for j in range(k):
            # print i, j
            name = 'part'
            if  i == len(s) - 1:
                name = 'noi_part'
            if i >= j:
                with open(folder + '/{}_{}_{}.in'.format(name, i, j), 'w') as f:
                    if j == 0:
                        with open(folder + '/part_{}_{}.in'.format(i - 1, j), 'r') as fr:
                            line = fr.readline()
                            # print type(line)
                            line1 = line.split("\n")
                            # print line
                            f.write(line1[0] + ' ' + str(s[i]) + '\n')
                    
                    elif j == i:
                        with open(folder + '/part_{}_{}.in'.format(i - 1, j - 1), 'r') as fr:
                            line = fr.readline()
                            # print type(line)
                            line1 = line.split("\n")
                            # print line
                            f.write(line1[0] + '\t' + str(s[i]) + '\n')
                        os.remove(folder + '/part_{}_{}.in'.format(i - 1, j - 1))
                    
                    elif j < i:
                        with open(folder + '/part_{}_{}.in'.format(i - 1, j - 1), 'r') as fr:
                            for line in fr:
                                # print line
                                line1 = line.split("\n")
                                # print line
                                f.write(line1[0] + '\t' + str(s[i]) + '\n')
                        os.remove(folder + '/part_{}_{}.in'.format(i - 1, j - 1))

                        with open(folder + '/part_{}_{}.in'.format(i - 1, j), 'r') as fr:
                            for line in fr:
                                # print type(line)
                                line1 = line.split("\n")
                                # print line
                                line2 = line1[0].split('\t')
                                # print line2
                                for l in range(j + 1):
                                    temp = line2[:]
                                    temp[l] = temp[l] + ' ' + str(s[i])
                                    line3 = "\t".join(temp)
                                    f.write(line3 +'\n')
                        if j == k - 1:
                            os.remove(folder + '/part_{}_{}.in'.format(i - 1, j))

def all_partitions_of_n_support(n, k):
    #Input: 1. A number to be broken into parts 2. Number of parts
    #Output: A list containing tuples of partitions

    if k == 1:
        return [[n]]

    output = []
    for i in range(n - k + 1, 0, -1):
        a = [i]
        exts = all_partitions_of_n_support(n - i, k - 1)
        for e in exts:
            temp = a[:]
            temp.extend(e)
            output.append(temp)
    return output

def all_partitions_of_n(n, folder):
    #Input: 1. A number to be broken into parts adding up to 
    # itself (Number of agents per partition) 2. Folder to store the files generated
    #Output: Files indexed by number of partitions
    name = '/agent_part_{}_'.format(n)
    name = folder + name
    for i in range(n):
        parts = all_partitions_of_n_support(n, i + 1)
        with open(name + '{}.in'.format(i), 'w') as f:
            for p in parts:
                data = ''
                for s in p:
                    data = data + str(s) + ' '
                data += '\n'
                f.write(data)

def tsp_support(g, source, nodes):
    if len(nodes) == 1:
        dist = nx.shortest_path_length(g, nodes[0], source, weight = 'length')
        return (dist, [nodes[0], source])
        
    min_len = np.inf
    seq = []
    for s in nodes:
        temp = [s]
        temp_n = nodes[:]
        temp_n.remove(s)
        (t_l, t_s) = tsp_support(g, source, temp_n)
        dist = nx.shortest_path_length(g, s, t_s[0], weight = 'length')
        if min_len > (dist + t_l):
            min_len = dist + t_l
            temp.extend(t_s)
            seq = temp
    return (min_len, seq)
        
def tsp_ideal_s(g, s):
    #Input: 1. A directed graph 2. A subset of nodes
    #Output: 1. Length of the shortest closed path through the subset of nodes 2. Sequence of nodes in the circuit
    if len(s) == 1:
        return (0, [s])
    l = np.inf
    p = []
    h = gh.nodes_complete_subgraph(g, s)
    for i in s:
        # print i
        temp = s[:]
        temp.remove(i)
        (t_l, t_s) = tsp_support(h, i, temp)
        dist = nx.shortest_path_length(h, i, t_s[0], weight = 'length')
        if l > (dist + t_l):
            l = dist + t_l
            p = [i]
            p.extend(t_s)

    circuit = [p[0]]
    for i in range(len(p) - 1):
        path = nx.shortest_path(g, p[i], p[i + 1])
        circuit.extend(path[1:])
    return (l, circuit)

def get_tp_given_p_s(g, p, s):
    #Input: 1. Directed Graph 2. Partition of Subset of Nodes 3. Number of agents per partition
    #Output: 1. Time Period 2. Dictionary with keys being partition and value being list of closed paths
    remaining_nodes = g.nodes()
    tp = 0.
    close_circuits = {}
    for i in range(len(p)):
        for j in p[i]:
            remaining_nodes.remove(j)
        t_l, t_p =  tsp_ideal_s(g, p[i])
        for j in t_p:
            if j in remaining_nodes:
                remaining_nodes.remove(j)
        close_circuits[i] = [t_p]
        if float(t_l)/s[i] > tp:
            tp = float(t_l)/s[i]

    while len(remaining_nodes) > 0:
        n = rn.sample(remaining_nodes, 1)
        cost = np.inf
        cc = []
        ind = 0
        for i in range(len(p)):
            temp = p[i][:]
            temp.extend(n)
            t_l, t_p = tsp_ideal_s(g, temp)
            if float(t_l)/s[i] < cost:
                cost = float(t_l)/s[i]
                cc = t_p
                ind = i
        close_circuits[ind].append(cc)
        for i in cc:
            if i in remaining_nodes:
                remaining_nodes.remove(i)
        if cost > tp:
            tp = cost
    return (tp, close_circuits)

# def min_tp_all_nodes(g, nodes, num_agents, folder):
#     #Input: 1. A directed graph 2. Nodes of Interest 3. Number of agents available
#     #Output: 1. Files corresponding to combination of partitions nodes and agents 2. File listing tp of all combinations 3. Minimum time period

#     min_tp = np.inf
#     min_part_s = []
#     min_part_a = []
#     min_circuits = {}

#     count = 0
#     all_partitions_of_s(nodes, num_agents, folder)
#     for i in range(num_agents):
#         agent_part = all_partitions_of_n(num_agents, i + 1)
#         with open(folder + '/part_s_{}_{}.in'.format(len(nodes) - 1, i), 'r') as fr:
