import os
import sys
import time
from random import random
check = False
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

sumo_startup = ['sumo-gui', '-c', '/home/meghdeep/Meghdeep/SysCon/sumo_rl/CAIR/CAIR.sumocfg']

traci.start(sumo_startup)
route_id = 'rou_'+str(random())
route_0 = ["142865547_3", "-142865547_3", "-142865547_2", "-142865549_2", "-142865549_1", "142865552_2", "-99829039_2"]
traci.route.add(route_id, route_0)
traci.vehicle.add(vehID = 'r0',routeID = route_id, typeID = "type1")
traci.vehicle.setStop(vehID = 'r0', edgeID = route_0[-1], duration = 2000.)
#print traci.vehicle.getIDList(), "here========="

print (traci.simulation.getMinExpectedNumber())
while traci.simulation.getMinExpectedNumber() > 0:
	traci.simulationStep()
	if 'r0' in traci.vehicle.getIDList():
		e = traci.vehicle.getRoute('r0')

		if traci.vehicle.isStopped('r0') and e[-1] == route_0[-1]:
			time.sleep(5)
			traci.vehicle.resume('r0')
			new_route = ['-99829039_2', '99829039_2', '99829039_9', '142865547_1']
			stop_pos = traci.lane.getLength(new_route[-1] + '_0')/2.0
			traci.vehicle.setRoute(vehID = 'r0', edgeList = new_route)
			#traci.vehicle.setStop(vehID = 'r0', edgeID = new_route[-1], duration = 2000.)
			traci.vehicle.setStop(vehID = 'r0', edgeID = new_route[-1],  pos=stop_pos, duration = 2000.)

		if traci.vehicle.isStopped('r0') and e[-1] != route_0[-1]:
			 traci.vehicle.resume('r0')
			 traci.vehicle.remove('r0', 0x03)
	else:
		time.sleep(5)
traci.close()
