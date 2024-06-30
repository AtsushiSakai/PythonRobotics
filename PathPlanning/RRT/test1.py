#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import time
import math
import sys
import pathlib
import threading
import matplotlib.pyplot as plt
import numpy as np
import argparse
import random
import matplotlib.animation as animation
sys.path.append(str(pathlib.Path(__file__).parent))

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
from rrt import RRT

show_animation = True
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode')
parser.add_argument('--connect1', help="Vehicle 1 connection target string.")
parser.add_argument('--connect2', help="Vehicle 2 connection target string.")
args = parser.parse_args()

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

connection_string1 = args.connect1 if args.connect1 else 'udpin:127.0.0.1:14552'
connection_string2 = args.connect2 if args.connect2 else 'udpin:127.0.0.1:14553'

print('Connecting to vehicle on:', connection_string1)
vehicle1 = connect(connection_string1, wait_ready=True)

print('Connecting to vehicle on:', connection_string2)
vehicle2 = connect(connection_string2, wait_ready=True)

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

def get_location_offset(location, dNorth, dEast):
    earth_radius = 6378137.0  # Radius of "spherical" earth
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * location.lat / 180))

    newlat = location.lat + (dLat * 180 / math.pi)
    newlon = location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, location.alt)

def follow_leader(follower, leader, follow_distance=5):
    while True:
        leader_location = leader.location.global_relative_frame
        follow_location = get_location_offset(leader_location, -follow_distance, 0)  # Follow behind the leader
        print(f"Follower moving to: {follow_location}")
        follower.simple_goto(follow_location)
        time.sleep(1)

def plot_destinations_and_fences(ax, destinations, fences):
    for dest in destinations:
        loc = dest['location']
        ax.scatter(loc.lon, loc.lat, color='blue', marker='o')
        ax.text(loc.lon + 0.001, loc.lat + 0.001, dest['name'], fontsize=8, color='blue')
    for fence in fences:
        ax.scatter(fence['lon'], fence['lat'], color='red', marker='o')
        circle = plt.Circle((fence['lon'], fence['lat']), fence['radius'], color='red', fill=False)
        ax.add_artist(circle)
        ax.text(fence['lon'] + 0.001, fence['lat'] + 0.001, fence['name'] + ' Fence', fontsize=8, color='red')

'''
def update_rrt_path(ax, drone_paths):
    ax.clear()
    plot_destinations_and_fences(ax, destinations, fences)
    for i, path in enumerate(drone_paths):
        if path:
            path_lons = [point.lon for point in path]
            path_lats = [point.lat for point in path]
            ax.plot(path_lons, path_lats, linestyle='-', marker='o', label=f'Drone {i+1} Path')
    # Add a dummy plot for the legend if no paths are present
    if not drone_paths:
        ax.plot([], [], label='No Path Available')  # This prevents the legend from being empty
    ax.legend()
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_title('Real-time RRT Path Visualization')
    ax.grid(True)
    plt.draw()
    plt.pause(0.1)
    
'''
def main():
    start_position = LocationGlobalRelative(51.73, 0.483, 45)  # Starting point for vehicle 2

    destinations = [
        {'name': 'Destination 1', 'location': LocationGlobalRelative(51.70715, 0.482, 37)},
        {'name': 'Destination 2', 'location': LocationGlobalRelative(51.7106, 0.4800, 56)},
        {'name': 'Destination 3', 'location': LocationGlobalRelative(51.7359071, 0.4732, 33)}
    ]

    fences = [
        {'name': 'Fence 1', 'lat': 51.71106, 'lon': 0.47100, 'radius': 0.0031477},  # Approx 530 meters
        {'name': 'Fence 2', 'lat': 51.719071, 'lon': 0.4772688, 'radius': 0.001471},  # Approx 525 meters
        {'name': 'Fence 3', 'lat': 51.725, 'lon': 0.485, 'radius': 0.00448}  # Approx 500 meters
    ]

    fig, ax = plt.subplots(figsize=(10, 10))  # Increase the figure size
    plt.ion()  # Turn on interactive mode
    fig2, ax1 = plt.subplots(figsize=(10, 8))
 
    plot_destinations_and_fences(ax1, destinations, fences)
    ax1.legend()
    plt.show()
    # Draw obstacles (fences)
    for fence in fences:
        circle = plt.Circle((fence['lat'], fence['lon']), fence['radius'], color='r', fill=False)
        ax.add_artist(circle)

    # Draw start position and destinations
    ax.plot(start_position.lat, start_position.lon, 'bo', label="Start Position")
    for destination in destinations:
        ax.plot(destination['location'].lat, destination['location'].lon, 'go', label=destination['name'])

    ax.legend()
    ax.set_aspect('equal')  # Set aspect ratio to be equal
    ax.set_xlim(51.68, 51.75)  # Set x-axis limits
    ax.set_ylim(0.46, 0.49)  # Set y-axis limits

    # ====Search Path with RRT====
    # Parameter
    obstacleList = [(f['lat'], f['lon'], f['radius']) for f in fences]
    
    # Add a path for the drone to follow from start_position to each destination
    waypoints = [start_position] + [d['location'] for d in destinations]

    all_smoothed_paths = []

    for i in range(len(waypoints) - 1):
        start = [waypoints[i].lat, waypoints[i].lon]
        goal = [waypoints[i + 1].lat, waypoints[i + 1].lon]
        rrt = RRT(start=start, goal=goal, rand_area=[-2, 15], obstacle_list=obstacleList)
        path = rrt.planning(animation=show_animation)

        if path is None:
            print(f"Path planning failed from {start} to {goal}")
            continue
        else:
            print(f"Path found from {start} to {goal}")

        # Path smoothing
        maxIter = 3000
        smoothedPath = path_smoothing(path, maxIter, obstacleList)
        all_smoothed_paths.append(smoothedPath)

    # Function to update the plot
    def update(num, data, line):
        line.set_data(data[:, :num])
        return line,

    ani_list = []

    for smoothedPath in all_smoothed_paths:
        # Draw final path
        if show_animation and smoothedPath:
            x_data = np.array([x for (x, y) in smoothedPath])
            y_data = np.array([y for (x, y) in smoothedPath])
            data = np.vstack((x_data, y_data))

            print(f"Smoothed Path Data: {data}")  # Debugging print statement

            line, = ax.plot([], [], '-c', label="Smoothed Path")
            ani = animation.FuncAnimation(fig, update, frames=len(x_data), fargs=(data, line), blit=True)
            ani_list.append(ani)

            plt.grid(True)
            plt.pause(0.01)  # Need for Mac

    plt.show()  # Keep the plot open


    # Command drones to follow smoothed paths in a loop
    def move_drones():
        while True:
            for smoothedPath in all_smoothed_paths:
                for point in smoothedPath:
                    location = LocationGlobalRelative(point[0], point[1], 47)  # Example altitude
                    print(f"Leader moving to: {location}")
                    vehicle2.simple_goto(location)
                    time.sleep(5)  # Adjust this delay as necessary to maintain distance

    if all_smoothed_paths:
        follower_thread = threading.Thread(target=follow_leader, args=(vehicle1, vehicle2))
        follower_thread.daemon = True
        follower_thread.start()

        move_drones_thread = threading.Thread(target=move_drones)
        move_drones_thread.start()
    



if __name__ == '__main__':
    main()

