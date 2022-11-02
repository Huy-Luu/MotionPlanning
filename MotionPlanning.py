from SlidingWindow import SlidingWindow
from StanleyController import StanleyController
import numpy as np
import matplotlib.pyplot as plt
from Vehicle import Vehicle
from UTMmodule import UTMmodule
from Point import OriginalPoint
from PathGenerator import PathGenerator
from StanleyController import StanleyController
from MQTTclient import MQTTclient
from SerialDataHandler import SerialDataHandler

# Initilization
vehicle = Vehicle(0.0, 0.0, 0.0, 1.388, 30)
serial_handler = SerialDataHandler('/dev/ttyUSB0', 115200)
scontroller = StanleyController()
sw = SlidingWindow(10)
path_generator = PathGenerator()
utm = UTMmodule()
client = MQTTclient("broker.hivemq.com", 1883, "SimulationCart")
client.init("control/auto")

og_points = []

# Waiting for incoming waypoints
while(client.waypointcame == False):
    pass
client.waypointcame = False

# Get the wp into a list
for i in range(0,len(client.waypointlist)):
    og_points.append(OriginalPoint(client.waypointlist[i]))
    #print(og_points[i].getLat())

path, yaw, waypoint_indices, offset = path_generator.generatePath(og_points, utm)

