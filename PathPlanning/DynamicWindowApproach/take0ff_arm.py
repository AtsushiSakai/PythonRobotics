#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
© Copyright 2015-2016, 3D Robotics.
guided_set_speed_yaw.py: (Copter Only)
This example shows how to move/direct Copter and send commands in GUIDED mode using DroneKit Python.
Example documentation: http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
"""
from __future__ import print_function
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import threading
import matplotlib.pyplot as plt
import argparse
from mpl_toolkits.mplot3d import Axes3D

# Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode')
parser.add_argument('--connect1', help="Vehicle 1 connection target string.")
parser.add_argument('--connect2', help="Vehicle 2 connection target string.")
args = parser.parse_args()

connection_string1 = args.connect1 if args.connect1 else 'udpin:127.0.0.1:14552'
connection_string2 = args.connect2 if args.connect2 else 'udpin:127.0.0.1:14553'

# Connect to the Vehicles
print('Connecting to vehicle on: %s' % connection_string1)
vehicle1 = connect(connection_string1, wait_ready=True)

print('Connecting to vehicle on: %s' % connection_string2)
vehicle2 = connect(connection_string2, wait_ready=True)

# Function to arm and takeoff
def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
  #   time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(3)
    vehicle.armed = True


    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
   # while True:
    print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
 #       if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
    print("Reached target altitude")
 #           break
    time.sleep(3)



# Arm and take off to altitude of 5 meters for both vehicles
arm_and_takeoff(vehicle1, 5)
time.sleep(10)  # Wait for some time to ensure vehicle1 has taken off
arm_and_takeoff(vehicle2, 5)
time.sleep(12)
print("Both vehicles are in the air.")
current_value1 = vehicle1.parameters['WPNAV_SPEED_UP']
current_value2 = vehicle2.parameters['WPNAV_SPEED_UP']
print(f"WPNAV_SPEED_UP parameter set to: {current_value1} for vehicle 1")
print(f"WPNAV_SPEED_UP parameter set to: {current_value2} for vehicle 2")

