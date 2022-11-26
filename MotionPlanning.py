from SlidingWindow import SlidingWindow
from StanleyController import StanleyController
import numpy as np
#import matplotlib.pyplot as plt
from Vehicle import Vehicle
from UTMmodule import UTMmodule
from Point import OriginalPoint
from PathGenerator import PathGenerator
from StanleyController import StanleyController
from MQTTclient import MQTTclient
from SerialDataHandler import SerialDataHandler
from PlanExecute import PlanExecute

# Initilization
vehicle = Vehicle(0.0, 0.0, 0.0, 1.388, 30)
serial_handler = SerialDataHandler('/dev/ttyClientB', 115200)
scontroller = StanleyController()
sw = SlidingWindow(10)
path_generator = PathGenerator()
utm = UTMmodule()
client = MQTTclient("broker.hivemq.com", 1883, "SimulationCart")
client.init("control/auto")

og_points = []

while True:
    # Waiting for incoming waypoints
    while(client.waypointcame == False):
        pass
    client.waypointcame = False
    print("Received")

    vehicle.x = 0
    vehicle.y = 0

    # Get the wp into a list
    for i in range(0,len(client.waypointlist)):
        og_points.append(OriginalPoint(client.waypointlist[i]))
        #print(og_points[i].getLat())

    path, yaw, waypoint_indices, offset = path_generator.generatePath(og_points, utm)

    path_x = []
    path_y = []
    waypoints = []
    waypoints_x = []
    waypoints_y = []

    for i in range(0, len(waypoint_indices)):
        waypoints.append(path[waypoint_indices[i] - 1])
        waypoints_x.append(waypoints[i].x)
        waypoints_y.append(waypoints[i].y)
        #print(waypoints[i].x, waypoints[i].y)


    for i in range(len(path)):
        path_x.append(path[i].x)
        path_y.append(path[i].y)

    # plt.plot(path_x, path_y, ".r", label="course")
    # plt.show()
    last_idx = len(path_x) -1
    target_idx, _ = scontroller.calcTargetIndex(vehicle, path, 0)
    print(target_idx)

    #Simulation.simulate(vehicle, dt, 500, 3, client, scontroller, sw, utm, target_idx, last_idx, path, waypoints, waypoint_indices, offset, yaw, True)

    PlanExecute.execute(
        client=client,
        serial_handler=serial_handler,
        vehicle=vehicle,
        scontroller=scontroller,
        sw=sw,
        utm=utm,
        target_idx=target_idx,
        last_idx=last_idx,
        og_points=og_points,
        path=path,
        waypoints=waypoints,
        waypoint_indices=waypoint_indices,
        offset=offset,
        yaw=yaw
    )

    og_points.clear()
