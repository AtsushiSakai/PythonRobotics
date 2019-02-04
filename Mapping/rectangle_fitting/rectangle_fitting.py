"""

Object shape recognition with rectangle fitting

author: Atsushi Sakai (@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import math
import numpy as np
import itertools
import random

show_animation = True


class Rectangle():

    def __init__(self):
        self.a = [None] * 4
        self.b = [None] * 4
        self.c = [None] * 4

        self.rect_c_x = [None] * 5
        self.rect_c_y = [None] * 5

    def plot(self):
        self.calc_rect_contour()
        plt.plot(self.rect_c_x, self.rect_c_y, "-r")

    def calc_rect_contour(self):

        self.rect_c_x[0], self.rect_c_y[0] = self.calc_cross_point(
            self.a[0:2], self.b[0:2], self.c[0:2])
        self.rect_c_x[1], self.rect_c_y[1] = self.calc_cross_point(
            self.a[1:3], self.b[1:3], self.c[1:3])
        self.rect_c_x[2], self.rect_c_y[2] = self.calc_cross_point(
            self.a[2:4], self.b[2:4], self.c[2:4])
        self.rect_c_x[3], self.rect_c_y[3] = self.calc_cross_point(
            [self.a[3], self.a[0]], [self.b[3], self.b[0]], [self.c[3], self.c[0]])
        self.rect_c_x[4], self.rect_c_y[4] = self.rect_c_x[0], self.rect_c_y[0]

    def calc_cross_point(self, a, b, c):
        x = (b[0] * -c[1] - b[1] * -c[0]) / (a[0] * b[1] - a[1] * b[0])
        y = (a[1] * -c[0] - a[0] * -c[1]) / (a[0] * b[1] - a[1] * b[0])
        return x, y


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
        plt.plot(self.x, self.y, ".b")

        # convert global coordinate
        gx, gy = self.calc_global_contour()
        plt.plot(gx, gy, "--b")

    def calc_global_contour(self):
        gx = [(ix * math.cos(self.yaw) + iy * math.sin(self.yaw))
              + self.x for (ix, iy) in zip(self.vc_x, self.vc_y)]
        gy = [(ix * math.sin(self.yaw) - iy * math.cos(self.yaw))
              + self.y for (ix, iy) in zip(self.vc_x, self.vc_y)]

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
            vr = math.hypot(vx, vy) * random.uniform(0.99, 1.01)

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


def calc_area_criterion(c1, c2):
    c1_max = max(c1)
    c2_max = max(c2)
    c1_min = min(c1)
    c2_min = min(c2)

    alpha = - (c1_max - c1_min) * (c2_max - c2_min)

    return alpha


def calc_closeness_criterion(c1, c2):
    c1_max = max(c1)
    c2_max = max(c2)
    c1_min = min(c1)
    c2_min = min(c2)

    D1 = [min([np.linalg.norm(c1_max - ic1),
               np.linalg.norm(ic1 - c1_min)]) for ic1 in c1]
    D2 = [min([np.linalg.norm(c2_max - ic2),
               np.linalg.norm(ic2 - c2_min)]) for ic2 in c2]

    d0 = 0.01
    beta = 0
    for i, _ in enumerate(D1):
        d = max(min([D1[i], D2[i]]), d0)
        beta += (1.0 / d)

    return beta


def rectangle_search(x, y):

    X = np.array([x, y]).T

    dtheta = np.deg2rad(0.5)
    minp = (-float('inf'), None)
    for theta in np.arange(0.0, math.pi / 2.0 - dtheta, dtheta):

        e1 = np.array([math.cos(theta), math.sin(theta)])
        e2 = np.array([-math.sin(theta), math.cos(theta)])

        c1 = X @ e1.T
        c2 = X @ e2.T

        # alpha = calc_area_criterion(c1, c2)
        beta = calc_closeness_criterion(c1, c2)

        # cost = alpha
        cost = beta

        if minp[0] < cost:
            minp = (cost, theta)

    # calc best rectangle
    sin_s = math.sin(minp[1])
    cos_s = math.cos(minp[1])

    c1_s = X @ np.array([cos_s, sin_s]).T
    c2_s = X @ np.array([-sin_s, cos_s]).T

    rect = Rectangle()
    rect.a[0] = cos_s
    rect.b[0] = sin_s
    rect.c[0] = min(c1_s)
    rect.a[1] = -sin_s
    rect.b[1] = cos_s
    rect.c[1] = min(c2_s)
    rect.a[2] = cos_s
    rect.b[2] = sin_s
    rect.c[2] = max(c1_s)
    rect.a[3] = -sin_s
    rect.b[3] = cos_s
    rect.c[3] = max(c2_s)

    return rect


def adoptive_range_segmentation(ox, oy):

    alpha = 0.2

    # Setup initial cluster
    S = []
    for i, _ in enumerate(ox):
        C = set()
        R = alpha * np.linalg.norm([ox[i], oy[i]])
        for j, _ in enumerate(ox):
            d = math.sqrt((ox[i] - ox[j])**2 + (oy[i] - oy[j])**2)
            if d <= R:
                C.add(j)
        S.append(C)

    # Merge cluster
    while 1:
        no_change = True
        for (c1, c2) in list(itertools.permutations(range(len(S)), 2)):
            if S[c1] & S[c2]:
                S[c1] = (S[c1] | S.pop(c2))
                no_change = False
                break
        if no_change:
            break

    return S


def main():

    # simulation parameters
    simtime = 30.0  # simulation time
    dt = 0.2  # time tick

    angle_reso = np.deg2rad(2.0)  # sensor angle resolution

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

        # step1: Adaptive Range Segmentation
        idsets = adoptive_range_segmentation(ox, oy)

        # step2 Rectangle search
        rects = []
        for ids in idsets:  # for each cluster
            cx = [ox[i] for i in range(len(ox)) if i in ids]
            cy = [oy[i] for i in range(len(oy)) if i in ids]
            rects.append(rectangle_search(cx, cy))

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.axis("equal")
            plt.plot(0.0, 0.0, "*r")
            v1.plot()
            v2.plot()

            # Plot range observation
            for ids in idsets:
                x = [ox[i] for i in range(len(ox)) if i in ids]
                y = [oy[i] for i in range(len(ox)) if i in ids]

                for (ix, iy) in zip(x, y):
                    plt.plot([0.0, ix], [0.0, iy], "-og")

                # plt.plot([ox[i] for i in range(len(ox)) if i in ids],
                # [oy[i] for i in range(len(ox)) if i in ids],
                # "o")
            for rect in rects:
                rect.plot()

            plt.pause(0.1)

    print("Done")


if __name__ == '__main__':
    main()
