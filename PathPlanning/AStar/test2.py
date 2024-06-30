#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
import threading
import sys
import pathlib
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

sys.path.append(str(pathlib.Path(__file__).parent))

show_animation = True
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode')
parser.add_argument('--connect1', help="Vehicle 1 connection target string.")
parser.add_argument('--connect2', help="Vehicle 2 connection target string.")
args = parser.parse_args()

class AStarPlanner:
    def __init__(self, ox, oy, resolution, rr):
        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(self.calc_xy_index(sx, self.min_x), self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x), self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        path = []
        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                path.append((self.calc_grid_position(goal_node.x, self.min_x), self.calc_grid_position(goal_node.y, self.min_y)))
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0], current.y + self.motion[i][1], current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node

            path.append((self.calc_grid_position(current.x, self.min_x), self.calc_grid_position(current.y, self.min_y)))

            # Animation
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

        rx, ry = self.calc_final_path(goal_node, closed_set)
        
        # Final path plot
        if show_animation:
            plt.plot(rx, ry, "-r")
            plt.pause(0.001)
        return rx, ry, path

    def calc_final_path(self, goal_node, closed_set):
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        return motion

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

def plot_destinations_and_fences(ax, start_position, destinations, fences):
    ax.plot(start_position.lon, start_position.lat, 'bo', label="Start Position")
    for dest in destinations:
        loc = dest['location']
        ax.plot(loc.lon, loc.lat, 'go')
        ax.text(loc.lon + 0.001, loc.lat + 0.001, dest['name'], fontsize=8, color='green')
    for fence in fences:
        ax.plot(fence['lon'], fence['lat'], 'ro')
        circle = plt.Circle((fence['lon'], fence['lat']), fence['radius'], color='red', fill=False)
        ax.add_artist(circle)
        ax.text(fence['lon'] + 0.001, fence['lat'] + 0.001, fence['name'] + ' Fence', fontsize=8, color='red')

def main():
    start_position = LocationGlobalRelative(51.73, 0.483, 45)  # Starting point for vehicle 2

    destinations = [  
         {'name': 'Destination 1', 'location': LocationGlobalRelative(51.70715, 0.482, 37)},  
         {'name': 'Destination 2', 'location': LocationGlobalRelative(51.725, 0.4700, 56)},  
         {'name': 'Destination 3', 'location': LocationGlobalRelative(51.7359071, 0.4732, 33)}  
     ]  

  

   

  

    fences = [  
         {'name': 'Fence 1', 'lat': 51.715, 'lon': 0.471500, 'radius': 0.0061477},  
         {'name': 'Fence 2', 'lat': 51.719071, 'lon': 0.4655, 'radius': 0.004471},  
         {'name': 'Fence 3', 'lat': 51.725, 'lon': 0.485, 'radius': 0.00448},
         {'name': 'Fence 4', 'lat': 51.72171, 'lon': 0.482, 'radius': 0.004471}   
     ] 

   
    fig2, ax = plt.subplots(figsize=(18, 18))  # Increase the figure size
    plt.ion()  # Turn on interactive mode

    plot_destinations_and_fences(ax, start_position, destinations, fences)

    ax.legend()
    ax.set_aspect('equal')  # Set aspect ratio to be equal
    ax.set_xlim(0.46, 0.49)  # Set x-axis limits
    ax.set_ylim(51.68, 51.75)  # Set y-axis limits

    ox = [f['lat'] for f in fences]
    oy = [f['lon'] for f in fences]

    # ====Search Path with A*====
    # Parameter
    grid_size = 0.0001  # 10 meters
    robot_radius = 0.0005  # 5 meters
    
    # Add a path for the drone to follow from start_position to each destination
    waypoints = [start_position] + [d['location'] for d in destinations]

    all_paths = []

    for i in range(len(waypoints) - 1):
        start = [waypoints[i].lat, waypoints[i].lon]
        goal = [waypoints[i + 1].lat, waypoints[i + 1].lon]
        a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
        rx, ry, path = a_star.planning(start[0], start[1], goal[0], goal[1])

        if not path:
            print(f"Path planning failed from {start} to {goal}")
            continue
        else:
            print(f"Path found from {start} to {goal}")

        all_paths.append(path)

    # Animate the paths
    for path in all_paths:
        x_data = [x for x, y in path]
        y_data = [y for x, y in path]
        ax.plot(x_data, y_data, "-r")
        plt.pause(0.004)
    plt.show()  # Keep the plot open

    # Command drones to follow paths in a loop
    def move_drones():
        while True:
            for path in all_paths:
                for point in path:
                    location = LocationGlobalRelative(point[0], point[1], 47)  # Example altitude
                    print(f"Leader moving to: {location}")
                    vehicle2.simple_goto(location)
                    time.sleep(5)  # Adjust this delay as necessary to maintain distance

    if all_paths:
        follower_thread = threading.Thread(target=follow_leader, args=(vehicle1, vehicle2))
        follower_thread.daemon = True
        follower_thread.start()

        move_drones_thread = threading.Thread(target=move_drones)
        move_drones_thread.start()

if __name__ == '__main__':
    main()

