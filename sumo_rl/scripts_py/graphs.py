#!/usr/bin/env python

import networkx as nx
import networkx.algorithms.shortest_paths.weighted as nxsh
import rospkg
import os
import math
import numpy as np

# def is_valid_partition(g, p):
#     #Input: 1. A directed graph 2. Partition of a subset of nodes
#     #Output: If the partition is valid or not
#     nodes = []
#     for i in p:
#         nodes.extend(i)
#     valid = True
#     for i in p:
#         temp = nodes[:]
#         for j in i:
#             temp.remove(j)
#         _, seq = tsp_ideal_s(g, i)
#         for k in temp:
#             if k in seq:
#                 valid = False
#                 return valid
#     return valid


def extract_from_in(path, undirected=True):

    g = nx.DiGraph()

    with open(path, 'r') as f:
        i = 0
        for line in f:
            if i == 0:
                n = map(int, line.split(' '))
                nodes = n[0]
                edges = n[1]
            elif i <= nodes:
                data = map(float, line.split(' '))
                g.add_node(i - 1)
                g.node[i - 1]['x'] = data[0]
                g.node[i - 1]['y'] = data[1]
            else:
                data = map(int, line.split(' '))
                length = math.sqrt((g.node[data[0]]['x'] - g.node[data[1]]['x'])**2 + (g.node[data[0]]['y'] - g.node[data[1]]['y'])**2)
                g.add_edge(data[0], data[1])
                g[data[0]][data[1]]['length'] = length
                if undirected:
                    g.add_edge(data[1], data[0])
                    g[data[1]][data[0]]['length'] = length
            i += 1
    return g

def get_arborescence(g, node):
    h = nx.DiGraph()
    h.add_nodes_from(g)
    for i in g.nodes():
        h.node[i] = g.node[i]
    _, paths = nxsh.single_source_dijkstra(g, node, weight = 'length')
    for p in paths:
        if len(paths[p]) > 1:
            for i in range(len(paths[p]) - 1):
                h.add_edge(paths[p][i], paths[p][i + 1])
                h.edge[paths[p][i]][paths[p][i + 1]] = g.edge[paths[p][i]][paths[p][i + 1]]
                h.add_edge(paths[p][i + 1], paths[p][i])
                h.edge[paths[p][i + 1]][paths[p][i]] = g.edge[paths[p][i + 1]][paths[p][i]]
    return h

def multi_root_forest_arborescence(g, nodes):
    #Input: 1. An undirected graph 2. Subset of nodes
    #Output: An undirected graph containing collection of arborescence rooted at the given subset of nodes and the set of 
    # nodes partitioned according to shortest distance to the root nodes (ties handled in the order of rooted node indices supplied)
    #NOTE: Assumes edge length in both directions is same  TODO: Relax this assumption
    h = nx.DiGraph()
    h.add_nodes_from(nodes)
    for i in h.nodes():
        h.node[i] = g.node[i]
    n = len(g.nodes())
    lens = []
    paths = []
    for i in nodes:
        l, p = nxsh.single_source_dijkstra(g, i, weight = 'length')
        lens.append(l)
        paths.append(p)
    for k in range(n):
        min_len = np.inf
        min_id = n
        for i in range(len(lens)):
            if lens[i][k] < min_len:
                min_len = lens[i][k]
                min_id = i
        path = paths[min_id][k]
        if len(path) > 1:
            for j in range(len(path) - 1):
                h.add_edge(path[j], path[j + 1])
                h.edge[path[j]][path[j + 1]] = g.edge[path[j]][path[j + 1]]
                h.edge[path[j + 1]][path[j]] = g.edge[path[j + 1]][path[j]]
                
    return h

def nodes_complete_subgraph(graph, nodes):
    #Input : 1. A Directed Graph 2. Subset of nodes 
    #Output : A complete directed graph with nodes as subset of nodes given and edge lengths as the shortest path length 
    h = nx.DiGraph()
    h.add_nodes_from(nodes)
    for i in nodes:
        h.node[i] = graph.node[i]
    for i in range(len(nodes) - 1):
        for j in range(i + 1, len(nodes)):
            h.add_edge(nodes[i], nodes[j])
            h.edge[nodes[i]][nodes[j]]['length'] = nx.shortest_path_length(graph, nodes[i], nodes[j])
            h.add_edge(nodes[j], nodes[i])
            h.edge[nodes[j]][nodes[i]]['length'] = nx.shortest_path_length(graph, nodes[j], nodes[i])
    return h