"""
Class for plotting a quadrotor

Author: Daniel Ingram (daniel-s-ingram)
"""

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


class Quadrotor():
    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, size=0.25, show_animation=True):
        self.p = np.array([
            [size / 2, -size / 2, 0, 0],
            [0, 0, size / 2, -size / 2],
            [0, 0, 0, 0],
            [1, 1, 1, 1]
        ])

        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.show_animation = show_animation

        if self.show_animation:
            plt.ion()
            fig = plt.figure()
            self.ax = fig.add_subplot(111, projection='3d')

        self.update_pose(x, y, z, roll, pitch, yaw)

    def update_pose(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

        if self.show_animation:
            self.plot()

    def transformation_matrix(self):
        sin_roll = np.sin(self.roll)
        cos_roll = np.cos(self.roll)
        sin_pitch = np.sin(self.pitch)
        cos_pitch = np.cos(self.pitch)
        sin_yaw = np.sin(self.yaw)
        cos_yaw = np.cos(self.yaw)

        return np.array(
            [[cos_yaw * cos_pitch, -sin_yaw * cos_roll + cos_yaw * sin_pitch * sin_roll,
              sin_yaw * sin_roll + cos_yaw * sin_pitch * cos_roll, self.x],
             [sin_yaw * cos_pitch, cos_yaw * cos_roll + sin_yaw * sin_pitch *
              sin_roll, -cos_yaw * sin_roll + sin_yaw * sin_pitch * cos_roll, self.y],
             [-sin_pitch, cos_pitch * sin_roll, cos_pitch * cos_yaw, self.z]
             ])

    def plot(self):
        T = self.transformation_matrix()
        P_T = T @ self.p

        plt.cla()

        self.ax.plot(P_T[0, :], P_T[1, :], P_T[2, :], 'k.')

        self.ax.plot(P_T[0, :2], P_T[1, :2], P_T[2, :2], 'r-')
        self.ax.plot(P_T[0, 2:], P_T[1, 2:], P_T[2, 2:], 'r-')

        self.ax.plot(self.x_data, self.y_data, self.z_data, 'b:')

        plt.xlim(-5, 5)
        plt.ylim(-5, 5)
        self.ax.set_zlim(0, 10)

        plt.pause(0.001)
