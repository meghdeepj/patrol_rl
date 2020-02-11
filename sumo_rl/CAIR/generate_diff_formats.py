#!/usr/bin/env python

import rospy
import sys
import rospkg
import os
import xml.etree.ElementTree as et
import networkx as nx
import math

def sumo_to_graphml(node_file, edge_file, graphml_file):

    graph = nx.DiGraph()

    node_parse = et.parse(node_file)
    node_root = node_parse.getroot()

    for n in node_root.iter('node'):
        graph.add_node(n.attrib['id'])
        graph.node[n.attrib['id']]['x'] = float(n.attrib['x'])
        graph.node[n.attrib['id']]['y'] = float(n.attrib['y'])

    edge_parse = et.parse(edge_file)
    edge_root = edge_parse.getroot()

    for e in edge_root.iter('edge'):
        n1 = e.attrib['from']
        n2 = e.attrib['to']
        graph.add_edge(n1, n2)
        graph.edge[n1][n2]['name'] = str(e.attrib['id'])
	graph.edge[n1][n2]['name'] = graph.edge[n1][n2]['name'].encode("utf-8")
        temp = e.attrib['shape'].strip()
        temp = temp.split(' ')
        temp1 = []
        temp2 = []
        for i in range(len(temp)):
            co_ord = temp[i].split(',')
            temp1.append(float(co_ord[0]))
            temp2.append(float(co_ord[1]))
        length = 0
        for i in range(1, len(temp)):
            length += math.sqrt((temp1[i] - temp1[i - 1]) ** 2 + (temp2[i] - temp2[i - 1]) ** 2)
        graph.edge[n1][n2]['length'] = length

    nx.write_graphml(graph, graphml_file)

def main(name):
    rospy.init_node('osm_to_all', anonymous = True)
    dir_name = rospkg.RosPack().get_path('pat_2020')
    osm_file = dir_name + '/graph_osm/' + name + '.osm'
    wbt_file = dir_name + '/graph_wbt/' + name + '.wbt'
    rndf_file = dir_name + '/graph_rndf/' + name + '.rndf'
    graphml_file = dir_name + '/graph_graphml/' + name + '.graphml'
    os.mkdir(dir_name + '/graph_sumo/' + name + '_net')
    sumo_folder = dir_name + '/graph_sumo/' + name + '_net'
    net_file = dir_name + '/graph_sumo/' + name + '_net/sumo.net.xml'
    nod_file = dir_name + '/graph_sumo/' + name + '_net/sumo.nod.xml'
    edg_file = dir_name + '/graph_sumo/' + name + '_net/sumo.edg.xml'
    
    #Create WBT file
    os.system('python /usr/local/webots/resources/osm_importer/importer.py --input={} --output={}'.format(osm_file, wbt_file))

    #Create SUMO files
    os.system('python /usr/local/webots/resources/sumo_exporter/exporter.py --input={} --output={}'.format(wbt_file, sumo_folder))
    os.system('/usr/local/webots/projects/default/resources/sumo/bin/netconvert --node-files={} --edge-files={} --output-file={}'.format(nod_file, edg_file, net_file))

    #Create Graphml
    sumo_to_graphml(nod_file, edg_file, graphml_file)

    #Create RNDF
    


    return


if __name__ == '__main__':
    if len(sys.argv[1:]) == 1:
        main(sys.argv[1])
    else:
        print 'Please pass the appropriate arguments'
