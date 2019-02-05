""" 

Simulator

author: Atsushi Sakai

"""

import numpy as np
import matplotlib.pyplot as plt
import math
import random


class VehicleSimulator():

    def __init__(self, ix, iy, iyaw, iv, max_v, w, L):
        self.x = ix
        self.y = iy
        self.yaw = iyaw
        self.v = iv
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
        gx = [(ix * np.cos(self.yaw) + iy * np.sin(self.yaw)) +
              self.x for (ix, iy) in zip(self.vc_x, self.vc_y)]
        gy = [(ix * np.sin(self.yaw) - iy * np.cos(self.yaw)) +
              self.y for (ix, iy) in zip(self.vc_x, self.vc_y)]

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

        self.vc_x, self.vc_y = self._interporate(self.vc_x, self.vc_y)

    def _interporate(self, x, y):
        rx, ry = [], []
        dtheta = 0.05
        for i in range(len(x) - 1):
            rx.extend([(1.0 - θ) * x[i] + θ * x[i + 1]
                       for θ in np.arange(0.0, 1.0, dtheta)])
            ry.extend([(1.0 - θ) * y[i] + θ * y[i + 1]
                       for θ in np.arange(0.0, 1.0, dtheta)])

        rx.extend([(1.0 - θ) * x[len(x) - 1] + θ * x[1]
                   for θ in np.arange(0.0, 1.0, dtheta)])
        ry.extend([(1.0 - θ) * y[len(y) - 1] + θ * y[1]
                   for θ in np.arange(0.0, 1.0, dtheta)])

        return rx, ry


class LidarSimulator():

    def __init__(self):
        self.range_noise = 0.01

    def get_observation_points(self, vlist, angle_reso):
        x, y, angle, r = [], [], [], []

        # store all points
        for v in vlist:

            gx, gy = v.calc_global_contour()

            for vx, vy in zip(gx, gy):
                vangle = math.atan2(vy, vx)
                vr = np.hypot(vx, vy) * random.uniform(1.0 - self.range_noise,
                                                       1.0 + self.range_noise)

                x.append(vx)
                y.append(vy)
                angle.append(vangle)
                r.append(vr)

        # ray casting filter
        rx, ry = self.ray_casting_filter(x, y, angle, r, angle_reso)

        return rx, ry

    def ray_casting_filter(self, xl, yl, thetal, rangel, angle_reso):
        rx, ry = [], []
        rangedb = [float("inf") for _ in range(
            int(np.floor((np.pi * 2.0) / angle_reso)) + 1)]

        for i in range(len(thetal)):
            angleid = int(round(thetal[i] / angle_reso))

            if rangedb[angleid] > rangel[i]:
                rangedb[angleid] = rangel[i]

        for i in range(len(rangedb)):
            t = i * angle_reso
            if rangedb[i] != float("inf"):
                rx.append(rangedb[i] * np.cos(t))
                ry.append(rangedb[i] * np.sin(t))

        return rx, ry


def main():
    print("start!!")

    print("done!!")


if __name__ == '__main__':
    main()
