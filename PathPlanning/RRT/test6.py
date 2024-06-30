"""
Path planning Sample Code with RRT with path smoothing

@author: AtsushiSakai(@Atsushi_twi)
"""

import math
import random
import matplotlib.pyplot as plt
import sys
import pathlib
import argparse
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

sys.path.append(str(pathlib.Path(__file__).parent))

from rrt import RRT

show_animation = True

def get_path_length(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.hypot(dx, dy)
        le += d

    return le

def get_target_point(path, targetL):
    le = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.hypot(dx, dy)
        le += d
        if le >= targetL:
            ti = i - 1
            lastPairLen = d
            break

    partRatio = (le - targetL) / lastPairLen

    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio

    return [x, y, ti]

def line_collision_check(first, second, obstacleList):
    # Line Equation
    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]

    try:
        a = y2 - y1
        b = -(x2 - x1)
        c = y2 * (x2 - x1) - x2 * (y2 - y1)
    except ZeroDivisionError:
        return False

    for (ox, oy, size) in obstacleList:
        d = abs(a * ox + b * oy + c) / (math.hypot(a, b))
        if d <= size:
            return False

    return True  # OK

def path_smoothing(path, max_iter, obstacle_list):
    le = get_path_length(path)

    for i in range(max_iter):
        # Sample two points
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        first = get_target_point(path, pickPoints[0])
        second = get_target_point(path, pickPoints[1])

        if first[2] <= 0 or second[2] <= 0:
            continue

        if (second[2] + 1) > len(path):
            continue

        if second[2] == first[2]:
            continue

        # collision check
        if not line_collision_check(first, second, obstacle_list):
            continue

        # Create New path
        newPath = []
        newPath.extend(path[:first[2] + 1])
        newPath.append([first[0], first[1]])
        newPath.append([second[0], second[1]])
        newPath.extend(path[second[2] + 1:])
        path = newPath
        le = get_path_length(path)

    return path

def connect_vehicle(connection_string):
    print('Connecting to vehicle on:', connection_string)
    return connect(connection_string, wait_ready=True)

def set_parameter_value(vehicle, parameter, value):
    msg = vehicle.message_factory.param_set_encode(
        0,  # Target system (0 for broadcast)
        0,  # Target component (0 for broadcast)
        parameter.encode('utf-8'),  # Parameter name as a byte string
        float(value),  # Parameter value
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32  # Parameter type
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def main():
    parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode')
    parser.add_argument('--connect1', help="Vehicle 1 connection target string.")
    parser.add_argument('--connect2', help="Vehicle 2 connection target string.")
    args = parser.parse_args()

    connection_string1 = args.connect1 if args.connect1 else 'udpin:127.0.0.1:14552'
    connection_string2 = args.connect2 if args.connect2 else 'udpin:127.0.0.1:14553'

    vehicle1 = connect_vehicle(connection_string1)
    vehicle2 = connect_vehicle(connection_string2)

    destinations = [
        {'name': 'Destination 1', 'location': LocationGlobalRelative(51.70715, 0.482, 37)},
        {'name': 'Destination 2', 'location': LocationGlobalRelative(51.7106, 0.4800, 56)},
        {'name': 'Destination 3', 'location': LocationGlobalRelative(51.7359071, 0.4732, 33)}
    ]

    fences = [  

        {'name': 'Fence 1', 'lat': 51.71106, 'lon': 0.47100, 'radius': 0.001477},  # Approx 530 meters 
        {'name': 'Fence 2', 'lat': 51.719071, 'lon': 0.4772688, 'radius': 0.001471},  # Approx 525 meters 
        {'name': 'Fence 3', 'lat': 51.725, 'lon': 0.485, 'radius': 0.00448}   # Approx 500 meters 
        #{'name': 'Fence 2', 'lat': 51.719071, 'lon': 0.4772688, 'radius': 0.001471},  # Approx 525 meters 
        #{'name': 'Fence 3', 'lat': 51.725, 'lon': 0.485, 'radius': 0.00448}   # Approx 500 meters 
    ]

    # ====Search Path with RRT====
    # Parameter
    obstacleList = [(f['lat'], f['lon'], f['radius']) for f in fences]
    rrt = RRT(start=[51.73, 0.483], goal=[51.7106, 0.4800],
              rand_area=[-50, 52], obstacle_list=obstacleList)
    path = rrt.planning(animation=show_animation)

    # Path smoothing
    maxIter = 3000
    smoothedPath = path_smoothing(path, maxIter, obstacleList)

    # Draw final path
    if show_animation:
        rrt.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')

        plt.plot([x for (x, y) in smoothedPath], [
            y for (x, y) in smoothedPath], '-c')

        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()

    # Command drones to follow smoothed path
    for point in smoothedPath:
        location = LocationGlobalRelative(point[0], point[1], 10)  # Example altitude
        vehicle2.simple_goto(location)
        time.sleep(2)  # Delay to ensure vehicle 1 moves first
        vehicle1.simple_goto(location)
        time.sleep(2)  # Adjust this delay as necessary to maintain distance

if __name__ == '__main__':
    main()

