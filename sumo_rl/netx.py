# NetworkX tutorial and tests

import itertools, copy
import networkx as nx
import pandas as pd
import matplotlib.pyplot as plt

G=nx.Graph(month='january')

print(G.nodes, G.edges)
print(G.graph)

# G.add_nodes_from([1,2,3])
# H=nx.path_graph(10)
# G.add_node(H)
# print(G.nodes(), H.nodes(), H.edges())
# G.clear()

G.add_edges_from([(1, 2), (1, 3)])
G.add_node(1)
G.add_edge(1, 2)
G.add_node("spam")
G.add_nodes_from("spam")
G.add_edge(3, 'm')
print(G.nodes, G.edges)
print(G.number_of_nodes(), G.number_of_edges())
print(list(G.adj['m']), G.degree[1])

G.remove_nodes_from('spam')
G.add_edge(1, 2)
G.add_edge(2,3)
print(G.nodes, G.edges)

G.nodes[1]['day']='mon'
G.nodes[3]['day']='tues'
G[1][2]['wt']=1.5
G.edges[1,3]['wt']=2.0

print(G.adj.items(), G.nodes.data())

DG = nx.DiGraph()
DG.add_weighted_edges_from([(1,2,0.5), (3,1, 0.75), (3,2, 1)])
print(DG.in_degree(1, weight='weight'), DG.degree(1, weight='weight'))
print(list(DG.neighbors(3)))

plt.subplot(121)
nx.draw(DG, with_labels=True, font_weight='bold')
plt.show()
#end of code
