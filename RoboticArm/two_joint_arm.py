"""
Inverse kinematics of a two-joint arm
Left-click the plot to set the goal position of the end effector

Author: Daniel Ingram (daniel-s-ingram)
"""
from __future__ import print_function, division

import matplotlib.pyplot as plt
import numpy as np

from math import sin, cos, atan2, acos, pi

Kp = 15
dt = 0.01

#Link lengths
l1 = l2 = 1

shoulder = np.array([0, 0])

#Set initial goal position to the initial end-effector position
x = 2
y = 0

plt.ion()

def two_joint_arm():
    """
    Computes the inverse kinematics for a planar 2DOF arm
    """
    theta1 = theta2 = 0
    while True:
        try:
            theta2_goal = acos((x**2 + y**2 - l1**2 -l2**2)/(2*l1*l2))
            theta1_goal = atan2(y, x) - atan2(l2*sin(theta2_goal), (l1 + l2*cos(theta2_goal)))

            if theta1_goal < 0:
                theta2_goal = -theta2_goal
                theta1_goal = atan2(y, x) - atan2(l2*sin(theta2_goal), (l1 + l2*cos(theta2_goal)))

            theta1 = theta1 + Kp*ang_diff(theta1_goal, theta1)*dt
            theta2 = theta2 + Kp*ang_diff(theta2_goal, theta2)*dt
        except ValueError as e:
            print("Unreachable goal")

        plot_arm(theta1, theta2, x, y)

def plot_arm(theta1, theta2, x, y):
    elbow = shoulder + np.array([l1*cos(theta1), l1*sin(theta1)])
    wrist = elbow + np.array([l2*cos(theta1+theta2), l2*sin(theta1+theta2)])
    plt.cla()
    plt.plot([shoulder[0], elbow[0]], [shoulder[1], elbow[1]], 'k-')
    plt.plot([elbow[0], wrist[0]], [elbow[1], wrist[1]], 'k-')
    plt.plot(shoulder[0], shoulder[1], 'ro')
    plt.plot(elbow[0], elbow[1], 'ro')
    plt.plot(wrist[0], wrist[1], 'ro')
    plt.plot([wrist[0], x], [wrist[1], y], 'g--')
    plt.plot(x, y, 'g*')
    plt.xlim(-2, 2)
    plt.ylim(-2, 2)
    plt.show()
    plt.pause(dt)

def ang_diff(theta1, theta2):
    #Returns the difference between two angles in the range -pi to +pi
    return (theta1-theta2+pi)%(2*pi)-pi

def click(event):
    global x, y
    x = event.xdata
    y = event.ydata

if __name__ == "__main__":
    fig = plt.figure()
    fig.canvas.mpl_connect("button_press_event", click)
    two_joint_arm()
