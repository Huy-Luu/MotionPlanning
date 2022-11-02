from turtle import position
from StanleyController import StanleyController
from SlidingWindow import SlidingWindow
from MQTTclient import MQTTclient
from Point import Point
import time as t

# Check steering mapping, sending data back to serial, check steering command method on old source

class PlanExecute:

    @staticmethod
    def execute(client, serial_handler, vehicle, scontroller, sw, utm, target_idx, last_idx, path, waypoints,  waypoint_indices, offset, yaw):
        count = 0
        wp_arr_flag = 0
        wp_no_arrived = 1
        wp_about_to_arrive = 1

        #First point
        serial_handler.send("t") #request first state
        position_lat, position_lon, yaw, v = serial_handler.receiveFourInputs()
        x, y = utm.fromLatLon(position_lat, position_lon)
        target_idx, _ = scontroller.calcTargetIndex(vehicle, path, 0)

        while True: # replace with a flag
            serial_handler.send("t")
            position_lat, position_lon, yaw, v = serial_handler.receiveFourInputs()
            x, y = utm.fromLatLon(position_lat, position_lon)
            di, target_idx = scontroller.stanleyControl(vehicle, path, yaw, target_idx)

            #update control
            di = scontroller.steeringMapping(di)

            point_to_send = Point(position_lat, position_lon)
            message = str(point_to_send.getY()) + "," + str(point_to_send.getX()) + "," + str(wp_arr_flag) + "," + str(wp_no_arrived) + "," + str(wp_about_to_arrive) + "," + "3.0" + "," + "180.0"
            client.publish(message, "data/position")

            if(target_idx > waypoint_indices[count]):
                    wp_arr_flag = 1
                    wp_no_arrived += 1
                    wp_about_to_arrive += 1
                    message = str(point_to_send.getY()) + "," + str(point_to_send.getX()) + "," + str(wp_arr_flag) + "," + str(wp_no_arrived) + "," + str(wp_about_to_arrive) + "," + "3.0" + "," + "180.0"
                    client.publish(message, "data/position")
                    print("REACHED WAYPOINT")
                    t.sleep(2)
                    count+=1