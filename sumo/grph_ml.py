# Reading graph_ml, accessing nodes, edges and functions

import itertools, copy
import networkx as nx
import pandas as pd
import matplotlib.pyplot as plt

G5=nx.read_graphml('./graph_ml/grid_5_5.graphml')

print('\nnodes: ', G5.nodes, '\nedges: ', G5.edges)
print('\nnodes.n: ', G5.number_of_nodes(), '\nedges.n: ', G5.number_of_edges())
#print(list(G5.adj['m']), G5.degree[1])
print('\nnodes.adj: ', G5.adj.items(), '\nnodes.data: ', G5.nodes.data())
#print('\nadj_matrix: ', nx.adjacency_matrix(G5))
plt.subplot(121)
nx.draw(G5, with_labels=True, font_weight='bold')
plt.show()

#end of code
