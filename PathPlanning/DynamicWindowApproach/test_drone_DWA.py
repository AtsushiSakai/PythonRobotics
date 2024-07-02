#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
import random
import queue
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
from dwa import Config, dwa_control, motion, RobotType, plot_robot

show_animation = True
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode')
parser.add_argument('--connect1', help="Vehicle 1 connection target string.")
parser.add_argument('--connect2', help="Vehicle 2 connection target string.")
args = parser.parse_args()
connection_string1 = args.connect1 if args.connect1 else 'udpin:127.0.0.1:14552'
connection_string2 = args.connect2 if args.connect2 else 'udpin:127.0.0.1:14553'

print('Connecting to vehicle on:', connection_string1)
vehicle1 = connect(connection_string1, wait_ready=True, timeout=60)
print('Connected to vehicle 1')

print('Connecting to vehicle on:', connection_string2)
vehicle2 = connect(connection_string2, wait_ready=True, timeout=60)
print('Connected to vehicle 2')

def get_location_offset(location, dNorth, dEast):
    earth_radius = 6378137.0  # Radius of "spherical" earth
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * location.lat / 180))

    newlat = location.lat + (dLat * 180 / math.pi)
    newlon = location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, location.alt)

def plot_destinations_and_fences(ax, start_position, destinations, fences):
    ax.plot(start_position.lon, start_position.lat, 'bo', markersize=10, label="Start Position")
    for dest in destinations:
        ax.plot(dest[1], dest[0], 'go', markersize=8, label='Destination')
        ax.text(dest[1] + 0.001, dest[0] + 0.001, 'Destination', fontsize=8, color='green')
    for fence in fences:
        ax.plot(fence['lon'], fence['lat'], 'ro', markersize=8, label='Fence')
        circle = plt.Circle((fence['lon'], fence['lat']), fence['radius'], color='red', fill=False)
        ax.add_artist(circle)
        ax.text(fence['lon'] + 0.001, fence['lat'] + 0.001, fence['name'] + ' Fence', fontsize=8, color='red')

def drone_movement(vehicle, config, vehicle_id, destinations, plot_queue):
    while True:
        destination = random.choice(destinations)
        goal = np.array([destination[0], destination[1]])
        ob = config.ob

        x = np.array([vehicle.location.global_relative_frame.lat, 
                      vehicle.location.global_relative_frame.lon, 
                      vehicle.attitude.yaw, 0.0, 0.0])  # Initial state

        while True:
            print(f"Drone {vehicle_id} current position: {x[0]:.6f}, {x[1]:.6f}, yaw: {x[2]:.6f}")
            # Call DWA control to get the control inputs and the trajectory
            u, trajectory = dwa_control(x, config, goal, ob)
            # Update the drone's state using the motion model
            x = motion(x, u, config.dt)
            
            next_location = LocationGlobalRelative(x[0], x[1], vehicle.location.global_relative_frame.alt)
            print(f"Drone {vehicle_id} moving to: {next_location.lat:.6f}, {next_location.lon:.6f}")
            vehicle.simple_goto(next_location)

            # Add data to plot_queue
            plot_queue.put((x[0], x[1], x[2], goal, ob, vehicle_id))

            # Check if the drone has reached its goal
            dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
            if dist_to_goal <= config.robot_radius:
                print(f"Drone {vehicle_id} reached destination at {destination}")
                break

            # Check if the drone is too close to any obstacle (fence)
            for obstacle in ob:
                dist_to_obstacle = math.hypot(x[0] - obstacle[0], x[1] - obstacle[1])
                if dist_to_obstacle <= config.robot_radius + 0.5:  # Safety threshold
                    print(f"Drone {vehicle_id} too close to an obstacle, recalculating path...")
                    break

        print(f"Drone {vehicle_id} selecting new destination...")

def plot_update(frame, plot_queue, ax, start_position, destinations, fences, config1, config2):
    while not plot_queue.empty():
        x, y, yaw, goal, ob, vehicle_id = plot_queue.get()
        ax.cla()
        plot_destinations_and_fences(ax, start_position, destinations, fences)
        plot_robot(x, y, yaw, config1 if vehicle_id == 1 else config2)
        ax.plot(goal[0], goal[1], "xr", label='Goal')
        ax.plot(ob[:, 1], ob[:, 0], "ok", label='Obstacle')
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_xlim(0.46, 0.49)  # Set x-axis limits to match the area
        ax.set_ylim(51.68, 51.75)  # Set y-axis limits to match the area
        ax.legend()


def main():


    start_position = LocationGlobalRelative(51.73, 0.483, 45)  # Starting point for visualization

    destinations = [
        [51.95376, 0.48471, 45],  # Springfield Hospital
        [51.73112, 0.4711, 56],  # Chelmsford & Essex Hospital
        [51.73602, 0.47358, 33],  # Fitzroy Surgery Chelmsford
        [51.743516, 0.47556, 45],  # Priory Hospital Chelmsford
        [51.77517, 0.46596, 56]  # Broomfield Hospital
    ]

    fences = [
        {'name': 'Fence 1', 'lat': 51.74531, 'lon': 0.47058, 'radius': 0.0031477},
        {'name': 'Fence 2', 'lat': 51.73484, 'lon': 0.45986, 'radius': 0.0041471},
        {'name': 'Fence 3', 'lat': 51.715071, 'lon': 0.4858, 'radius': 0.0041471},
        {'name': 'Fence 4', 'lat': 51.75032, 'lon': 0.48405, 'radius': 0.00448},
        {'name': 'Fence 5', 'lat': 51.74856, 'lon': 0.48624, 'radius': 0.00448},
        {'name': 'Fence 6', 'lat': 51.73793, 'lon': 0.48606, 'radius': 0.0041471},
        {'name': 'Fence 7', 'lat': 51.76216, 'lon': 0.47748, 'radius': 0.00448},
        {'name': 'Fence 8', 'lat': 51.76928, 'lon': 0.46855, 'radius': 0.0041471}
    ]
    # Convert fences to obstacles for the DWA config
    obstacles = []
    for fence in fences:
        obstacles.append([fence['lat'], fence['lon']])

    config1 = Config()
    config1.robot_type = RobotType.circle
    config1.ob = np.array(obstacles)

    config2 = Config()
    config2.robot_type = RobotType.circle
    config2.ob = np.array(obstacles)

    plot_queue = queue.Queue()

    vehicle1_thread = threading.Thread(target=drone_movement, args=(vehicle1, config1, 1, destinations, plot_queue))
    vehicle1_thread.daemon = True
    vehicle1_thread.start()

    vehicle2_thread = threading.Thread(target=drone_movement, args=(vehicle2, config2, 2, destinations, plot_queue))
    vehicle2_thread.daemon = True
    vehicle2_thread.start()

    fig, ax = plt.subplots(figsize=(10, 10))  # Increase the figure size
    ani = animation.FuncAnimation(fig, plot_update, fargs=(plot_queue, ax, start_position, destinations, fences, config1, config2), interval=100, cache_frame_data=False)

    plt.show()

if __name__ == '__main__':
    main()

