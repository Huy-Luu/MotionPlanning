#from turtle import position
from StanleyController import StanleyController
from SlidingWindow import SlidingWindow
from MQTTclient import MQTTclient
from Point import Point
from SteeringMappingSender import SteeingMappingSender
import numpy as np
import time as t

# Check steering mapping, sending data back to serial, check steering command method on old source

class PlanExecute:

    @staticmethod
    def execute(client, serial_handler, vehicle, scontroller, sw, utm, target_idx, last_idx, og_points, path, waypoints,  waypoint_indices, offset, yaw):
        count = 0
        wp_arr_flag = 0
        wp_no_arrived = 1
        wp_about_to_arrive = 1
        f = open(r"/home/hluu/MotionPlanning/data.txt", "a")
        f.write("Waypoints" + '\r')
        for i in range(0, len(og_points)):
            f.write(str(og_points[i].getLat()) + str(og_points[i].getLon()) + '\r')
        f.write("Positions" + '\r')

        #First point
        serial_handler.send("t") #request first state
        position_lat, position_lon, yaw, v = serial_handler.receiveFourInputs()
        x, y = utm.fromLatlon(position_lat, position_lon)
        vehicle.x = x
        vehicle.y = y
        vehicle.yaw = np.radians(yaw)
        target_idx, _ = scontroller.calcTargetIndex(vehicle, path, 0)
        serial_handler.send("w") # run the cart
        t.sleep(0.1)

        while last_idx > target_idx:
            serial_handler.send("t")
            position_lat, position_lon, yaw, v = serial_handler.receiveFourInputs()
            x, y = utm.fromLatlon(position_lat, position_lon)
            vehicle.x = x - offset.x
            vehicle.y = y - offset.y
            vehicle.yaw = np.radians(yaw)
            di, target_idx = scontroller.stanleyControl(vehicle, path, yaw, target_idx)
            SteeingMappingSender.sendMapped(serial_handler, di)
            f.write(str(position_lat) + "," + str(position_lon) + ","  + str(vehicle.x) + "," + str(vehicle.y) + "," + str(vehicle.v) + "," + str(vehicle.yaw) + ","  + '\r')

            #update control
            di = scontroller.steeringMapping(di)

            point_to_send = Point(position_lat, position_lon)
            message = str(point_to_send.getY()) + "," + str(point_to_send.getX()) + "," + str(wp_arr_flag) + "," + str(wp_no_arrived) + "," + str(wp_about_to_arrive) + "," + "3.0" + "," + "180.0"
            client.publish(message, "data/position")

            if(target_idx > waypoint_indices[count]):
                    serial_handler.send("f") # stop when arriving at the waypoint
                    wp_arr_flag = 1
                    wp_no_arrived += 1
                    wp_about_to_arrive += 1
                    message = str(point_to_send.getY()) + "," + str(point_to_send.getX()) + "," + str(wp_arr_flag) + "," + str(wp_no_arrived) + "," + str(wp_about_to_arrive) + "," + "3.0" + "," + "180.0"
                    client.publish(message, "data/position")
                    print("REACHED WAYPOINT")
                    t.sleep(5)
                    serial_handler.send("w") # continue running the cart
                    count+=1

        wp_arr_flag = 1
        wp_no_arrived += 1
        wp_about_to_arrive += 1
        serial_handler.send("f") # Stop the cart
        message = str(point_to_send.getY()) + "," + str(point_to_send.getX()) + "," + str(wp_arr_flag) + "," + str(wp_no_arrived) + "," + str(wp_about_to_arrive) + "," + "3.0" + "," + "180.0"
        client.publish(message, "data/position")
        f.write(str(position_lat) + "," + str(position_lon) + "," + str(vehicle.v) + "," + str(vehicle.yaw) + ","  + '\r')
