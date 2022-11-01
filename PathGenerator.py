
from pathlib import Path
from UTMmodule import UTMmodule
import math
import numpy as np
from Point import OriginalPoint
from Point import Point

class PathGenerator():

    def line(p0, p1):
        a = (p0.y - p1.y)/(p0.x - p1.x)
        b = p0.y - a*p0.x
        return a,b

    def calcPerpendicular(p,a,b):
        a_p = -1/a
        b_p = p.y - a_p * p.x
        return a_p, b_p

    def intersection(a1,b1,a2,b2):
        x = (b1-b2)/(a2-a1)
        y = a1 * x + b1
        p = Point(x, y)
        return p

    def distance(p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def vectorAngle(center, p):
        return math.atan2(p.y - center.y, p.x - center.x)

    def printPoint(p):
        print(p.x, p.y)

    @staticmethod
    def generatePath(og_points, utm):
        cv_points = []

        for i in range (0,len(og_points)):
            #print(og_points[i].getLat(), og_points[i].getLon())
            x_temp, y_temp = utm.fromLatlon(og_points[i].getLat(), og_points[i].getLon())
            cv_points.append(Point(x_temp, y_temp))

        offset = Point(cv_points[0].x, cv_points[0].y)
        
            
        #offset
        for i in range(0,len(cv_points)):
            cv_points[i].x = cv_points[i].x - offset.x
            cv_points[i].y = cv_points[i].y - offset.y
            #PathGenerator.printPoint(cv_points[i])
            #print(cv_points[i].x, cv_points[i].y)

        line = []
        path = []
        ap = []
        bp = []
        end_point_each_segments = []
        arc = []

        #draw lines
        for i in range(0,len(cv_points)-1):
            a, b = PathGenerator.line(cv_points[i], cv_points[i+1])
            #print(a,b)
            #length = math.sqrt((x[i]-x[i+1])**2 + (y[i]- y[i+1])**2)
            length = PathGenerator.distance(cv_points[i], cv_points[i+1])
            segments = int(length * 10)
            x_delta = (cv_points[i+1].x - cv_points[i].x)/ segments
            y_delta = (cv_points[i+1].y - cv_points[i].y)/ segments
            for j in range(0, segments):
                line.append(Point(cv_points[i].x + j * x_delta, cv_points[i].y + j * y_delta))
                #PathGenerator.printPoint(line[j])
            
            path = np.append(path, line)
        
            #for i in range(0, len(path)):
                #PathGenerator.printPoint(path[i])

            end_point_each_segments.append(len(path)) # as the name suggested

            a_p, b_p = PathGenerator.calcPerpendicular(line[10], a, b)
            ap.append(a_p)
            bp.append(b_p)

            arc.append(Point(line[10].x, line[10].y))

            a_p, b_p = PathGenerator.calcPerpendicular(line[len(line) - 10], a, b)
            ap.append(a_p)
            bp.append(b_p)

            arc.append(Point(line[len(line) - 10].x, line[len(line) - 10].y))
            #for i in range(0, len(arc)):
            #    PathGenerator.printPoint(arc[i])

            line.clear()

        centers = []
        radiuses = []
        angles = []
        for i in range(1, len(ap) - 2, 2):
            #print(ap[i])
            center = PathGenerator.intersection(ap[i], bp[i], ap[i+1], bp[i+1])
            centers.append(center)
            radius = PathGenerator.distance(center, arc[i])
            radiuses.append(radius)

            angle = PathGenerator.vectorAngle(center, arc[i])
            print(angle)
            angles.append(angle)
            angle = PathGenerator.vectorAngle(center, arc[i+1])
            print(angle)
            angles.append(angle)

        path_arc = []

        print("Angles are: " + str(angles))

        for i in range(0, len(angles), 2):
            delta = angles[i+1] - angles[i]

            if(abs(delta) > math.radians(180)):
                if(angles[i]>0):
                    angles[i+1] = angles[i+1] + math.radians(360)
                    delta = angles[i+1] - angles[i]

                if(angles[i]<0):
                    angles[i+1] = angles[i+1] - math.radians(360)
                    delta = angles[i+1] - angles[i]

            segment = delta/20

            current_step = angles[i]
            for j in range(0, 20):
                p_arc_x = radiuses[int(i/2)]*math.cos(current_step) + centers[int(i/2)].x
                p_arc_y = radiuses[int(i/2)]*math.sin(current_step) + centers[int(i/2)].y
                path_arc.append(Point(p_arc_x, p_arc_y))
                current_step += segment

        # for i in range(0, len(path)):
        #     PathGenerator.printPoint(path[i])

        print("End point each segments:")
        print(end_point_each_segments)

        # for i in range(0,len(end_point_each_segments)-1):
        #     path[end_point_each_segments[i]-10: end_point_each_segments[i]+10].x = path_arc[20*i: 20*i+20].x
        #     path[end_point_each_segments[i]-10: end_point_each_segments[i]+10].y = path_arc[20*i: 20*i+20].y

        for i in range(0,len(end_point_each_segments)-1):
            count = 0 + i *20
            for j in range(end_point_each_segments[i]-10, end_point_each_segments[i]+10):
                path[j].x = path_arc[count].x
                path[j].y = path_arc[count].y
                count += 1
            
        yaw = []
        for i in range(0, len(path) - 1):
            yaw_temp = math.atan2(path[i+1].y - path[i].y, path[i+1].x - path[i].x)
            yaw.append(yaw_temp)

        yaw_temp = yaw[len(yaw)-1]
        yaw.append(yaw_temp)

        return path, yaw, end_point_each_segments, offset