"""

Object shape recognition with rectangle fitting

author: Atsushi Sakai (@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import math
import random
import numpy as np

show_animation = True


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
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += omega * dt
        self.v += a * dt
        if self.v >= self.max_v:
            self.v = self.max_v

    def plot(self):
        plt.plot(self.x, self.y, ".r")

        # convert global coordinate
        gx, gy = self.calc_global_contour()
        plt.plot(gx, gy, "-xr")

    def calc_global_contour(self):
        gx = [(ix * math.cos(self.yaw) + iy * math.sin(self.yaw)) +
              self.x for (ix, iy) in zip(self.vc_x, self.vc_y)]
        gy = [(ix * math.sin(self.yaw) - iy * math.cos(self.yaw)) +
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


def get_observation_points(vlist, angle_reso):
    x, y, angle, r = [], [], [], []

    # store all points
    for v in vlist:

        gx, gy = v.calc_global_contour()

        for vx, vy in zip(gx, gy):
            vangle = math.atan2(vy, vx)
            vr = math.hypot(vx, vy)  # * random.uniform(0.95, 1.05)

            x.append(vx)
            y.append(vy)
            angle.append(vangle)
            r.append(vr)

    # ray casting filter
    rx, ry = ray_casting_filter(x, y, angle, r, angle_reso)

    return rx, ry


def ray_casting_filter(xl, yl, thetal, rangel, angle_reso):
    rx, ry = [], []
    rangedb = [float("inf") for _ in range(
        int(math.floor((math.pi * 2.0) / angle_reso)) + 1)]

    for i in range(len(thetal)):
        angleid = int(round(thetal[i] / angle_reso))

        if rangedb[angleid] > rangel[i]:
            rangedb[angleid] = rangel[i]

    for i in range(len(rangedb)):
        t = i * angle_reso
        if rangedb[i] != float("inf"):
            rx.append(rangedb[i] * math.cos(t))
            ry.append(rangedb[i] * math.sin(t))

    return rx, ry


def main():

    # simulation parameters
    simtime = 30.0  # simulation time
    dt = 0.2  # time tick

    angle_reso = np.deg2rad(3.0)  # sensor angle resolution

    v1 = VehicleSimulator(-10.0, 0.0, np.deg2rad(90.0),
                          0.0, 50.0 / 3.6, 3.0, 5.0)
    v2 = VehicleSimulator(20.0, 10.0, np.deg2rad(180.0),
                          0.0, 50.0 / 3.6, 4.0, 10.0)

    time = 0.0
    while time <= simtime:
        time += dt

        v1.update(dt, 0.1, 0.0)
        v2.update(dt, 0.1, -0.05)

        ox, oy = get_observation_points([v1, v2], angle_reso)

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.axis("equal")
            plt.plot(0.0, 0.0, "*r")
            v1.plot()
            v2.plot()

            plt.plot(ox, oy, "ob")
            # plt.plot(x, y, "xr")
            # plot_circle(ex, ey, er, "-r")
            plt.pause(0.1)

    print("Done")


if __name__ == '__main__':
    main()
