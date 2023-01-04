""" 

Simulator

author: Atsushi Sakai

"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import numpy as np
import matplotlib.pyplot as plt
import math
import random

from utils.angle import rot_mat_2d


class VehicleSimulator:

    def __init__(self, i_x, i_y, i_yaw, i_v, max_v, w, L):
        self.x = i_x
        self.y = i_y
        self.yaw = i_yaw
        self.v = i_v
        self.max_v = max_v
        self.W = w
        self.L = L
        self._calc_vehicle_contour()

    def update(self, dt, a, omega):
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += omega * dt
        self.v += a * dt
        if self.v >= self.max_v:
            self.v = self.max_v

    def plot(self):
        plt.plot(self.x, self.y, ".b")

        # convert global coordinate
        gx, gy = self.calc_global_contour()
        plt.plot(gx, gy, "--b")

    def calc_global_contour(self):
        gxy = np.stack([self.vc_x, self.vc_y]).T @ rot_mat_2d(self.yaw)
        gx = gxy[:, 0] + self.x
        gy = gxy[:, 1] + self.y

        return gx, gy

    def _calc_vehicle_contour(self):

        self.vc_x = []
        self.vc_y = []

        self.vc_x.append(self.L / 2.0)
        self.vc_y.append(self.W / 2.0)

        self.vc_x.append(self.L / 2.0)
        self.vc_y.append(-self.W / 2.0)

        self.vc_x.append(-self.L / 2.0)
        self.vc_y.append(-self.W / 2.0)

        self.vc_x.append(-self.L / 2.0)
        self.vc_y.append(self.W / 2.0)

        self.vc_x.append(self.L / 2.0)
        self.vc_y.append(self.W / 2.0)

        self.vc_x, self.vc_y = self._interpolate(self.vc_x, self.vc_y)

    @staticmethod
    def _interpolate(x, y):
        rx, ry = [], []
        d_theta = 0.05
        for i in range(len(x) - 1):
            rx.extend([(1.0 - theta) * x[i] + theta * x[i + 1]
                       for theta in np.arange(0.0, 1.0, d_theta)])
            ry.extend([(1.0 - theta) * y[i] + theta * y[i + 1]
                       for theta in np.arange(0.0, 1.0, d_theta)])

        rx.extend([(1.0 - theta) * x[len(x) - 1] + theta * x[1]
                   for theta in np.arange(0.0, 1.0, d_theta)])
        ry.extend([(1.0 - theta) * y[len(y) - 1] + theta * y[1]
                   for theta in np.arange(0.0, 1.0, d_theta)])

        return rx, ry


class LidarSimulator:

    def __init__(self):
        self.range_noise = 0.01

    def get_observation_points(self, v_list, angle_resolution):
        x, y, angle, r = [], [], [], []

        # store all points
        for v in v_list:

            gx, gy = v.calc_global_contour()

            for vx, vy in zip(gx, gy):
                v_angle = math.atan2(vy, vx)
                vr = np.hypot(vx, vy) * random.uniform(1.0 - self.range_noise,
                                                       1.0 + self.range_noise)

                x.append(vx)
                y.append(vy)
                angle.append(v_angle)
                r.append(vr)

        # ray casting filter
        rx, ry = self.ray_casting_filter(angle, r, angle_resolution)

        return rx, ry

    @staticmethod
    def ray_casting_filter(theta_l, range_l, angle_resolution):
        rx, ry = [], []
        range_db = [float("inf") for _ in range(
            int(np.floor((np.pi * 2.0) / angle_resolution)) + 1)]

        for i in range(len(theta_l)):
            angle_id = int(round(theta_l[i] / angle_resolution))

            if range_db[angle_id] > range_l[i]:
                range_db[angle_id] = range_l[i]

        for i in range(len(range_db)):
            t = i * angle_resolution
            if range_db[i] != float("inf"):
                rx.append(range_db[i] * np.cos(t))
                ry.append(range_db[i] * np.sin(t))

        return rx, ry
