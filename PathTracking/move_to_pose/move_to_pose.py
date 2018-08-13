"""
Move to specified pose
Author: Daniel Ingram (daniel-s-ingram)

P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7
"""

from __future__ import print_function, division

import matplotlib.pyplot as plt
import numpy as np

from math import cos, sin, sqrt, atan2, pi
from random import random

Kp_rho = 3
Kp_alpha = 5
Kp_beta = -2
dt = 0.01

#Corners of triangular vehicle when pointing to the right (0 radians)
p1_i = np.array([0.5, 0, 1]).T
p2_i = np.array([-0.5, 0.25, 1]).T
p3_i = np.array([-0.5, -0.25, 1]).T

x_traj = []
y_traj = []

plt.ion()

def move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
    """
    rho is the distance between the robot and the goal position
    alpha is the angle to the goal relative to the heading of the robot
    beta is the angle between the robot's position and the goal position plus the goal angle

    Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards the goal
    Kp*beta*beta rotates the line so that it is parallel to the goal angle
    """
    x = x_start
    y = y_start
    theta = theta_start
    while True:
        x_diff = x_goal - x
        y_diff = y_goal - y

        rho = sqrt(x_diff**2 + y_diff**2)
        alpha = atan2(y_diff, x_diff) - theta
        beta = theta_goal - theta - alpha

        v = Kp_rho*rho
        w = Kp_alpha*alpha + Kp_beta*beta

        if alpha > pi/2 or alpha < -pi/2:
            v = -v

        theta = theta + w*dt
        x = x + v*cos(theta)*dt
        y = y + v*sin(theta)*dt
        x_traj.append(x)
        y_traj.append(y)

        plot_vehicle(x, y, theta, x_start, y_start, x_goal, y_goal, theta_goal, x_traj, y_traj)


def plot_vehicle(x, y, theta, x_start, y_start, x_goal, y_goal, theta_goal, x_traj, y_traj):
    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)
    plt.cla()
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')
    plt.plot(x_start, y_start, 'r*')
    plt.arrow(x_goal, y_goal, cos(theta_goal), sin(theta_goal), color='g', width=0.1)
    plt.plot(x_traj, y_traj, 'b--')
    plt.xlim(0, 20)
    plt.ylim(0, 20)
    plt.show()
    plt.pause(dt/10)

def transformation_matrix(x, y, theta):
    return np.array([
        [cos(theta), -sin(theta), x],
        [sin(theta), cos(theta), y],
        [0, 0, 1]
        ])

if __name__ == '__main__':
    x_start = 4 #20*random()
    y_start = 15 #20*random()
    theta_start = pi/2 #2*pi*random() - pi
    x_goal = 13 #20*random()
    y_goal = 3 #20*random()
    theta_goal = -pi/2 #2*pi*random() - pi
    print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" % (x_start, y_start, theta_start))
    print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" % (x_goal, y_goal, theta_goal))
    move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)