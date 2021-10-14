"""

Object shape recognition with L-shape fitting

author: Atsushi Sakai (@Atsushi_twi)

Ref:
- Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners -
The Robotics Institute Carnegie Mellon University
https://www.ri.cmu.edu/publications/
efficient-l-shape-fitting-for-vehicle-detection-using-laser-scanners/

"""

import matplotlib.pyplot as plt
import numpy as np
import itertools
from enum import Enum
from scipy.spatial.transform import Rotation as Rot

from Mapping.rectangle_fitting.simulator \
    import VehicleSimulator, LidarSimulator

show_animation = True


class LShapeFitting:

    class Criteria(Enum):
        AREA = 1
        CLOSENESS = 2
        VARIANCE = 3

    def __init__(self):
        # Parameters
        self.criteria = self.Criteria.VARIANCE
        self.min_dist_of_closeness_criteria = 0.01  # [m]
        self.d_theta_deg_for_search = 1.0  # [deg]
        self.R0 = 3.0  # [m] range segmentation param
        self.Rd = 0.001  # [m] range segmentation param

    def fitting(self, ox, oy):

        # step1: Adaptive Range Segmentation
        id_sets = self._adoptive_range_segmentation(ox, oy)

        # step2 Rectangle search
        rects = []
        for ids in id_sets:  # for each cluster
            cx = [ox[i] for i in range(len(ox)) if i in ids]
            cy = [oy[i] for i in range(len(oy)) if i in ids]
            rects.append(self._rectangle_search(cx, cy))

        return rects, id_sets

    @staticmethod
    def _calc_area_criterion(c1, c2):
        c1_max = max(c1)
        c2_max = max(c2)
        c1_min = min(c1)
        c2_min = min(c2)

        alpha = -(c1_max - c1_min) * (c2_max - c2_min)

        return alpha

    def _calc_closeness_criterion(self, c1, c2):
        c1_max = max(c1)
        c2_max = max(c2)
        c1_min = min(c1)
        c2_min = min(c2)

        # Vectorization
        D1 = np.minimum(c1_max - c1, c1 - c1_min)
        D2 = np.minimum(c2_max - c2, c2 - c2_min)
        d = np.maximum(np.minimum(D1, D2), self.min_dist_of_closeness_criteria)
        beta = (1.0 / d).sum()

        return beta

    @staticmethod
    def _calc_variance_criterion(c1, c2):
        c1_max = max(c1)
        c2_max = max(c2)
        c1_min = min(c1)
        c2_min = min(c2)

        # Vectorization
        D1 = np.minimum(c1_max - c1, c1 - c1_min)
        D2 = np.minimum(c2_max - c2, c2 - c2_min)
        E1 = D1[D1 < D2]
        E2 = D2[D1 >= D2]
        V1 = - np.var(E1) if len(E1) > 0 else 0.
        V2 = - np.var(E2) if len(E2) > 0 else 0.
        gamma = V1 + V2

        return gamma

    def _rectangle_search(self, x, y):

        X = np.array([x, y]).T

        d_theta = np.deg2rad(self.d_theta_deg_for_search)
        min_cost = (-float('inf'), None)
        for theta in np.arange(0.0, np.pi / 2.0 - d_theta, d_theta):

            rot = Rot.from_euler('z', theta).as_matrix()[0:2, 0:2]
            c = X @ rot
            c1 = c[:, 0]
            c2 = c[:, 1]

            # Select criteria
            cost = 0.0
            if self.criteria == self.Criteria.AREA:
                cost = self._calc_area_criterion(c1, c2)
            elif self.criteria == self.Criteria.CLOSENESS:
                cost = self._calc_closeness_criterion(c1, c2)
            elif self.criteria == self.Criteria.VARIANCE:
                cost = self._calc_variance_criterion(c1, c2)

            if min_cost[0] < cost:
                min_cost = (cost, theta)

        # calc best rectangle
        sin_s = np.sin(min_cost[1])
        cos_s = np.cos(min_cost[1])

        c1_s = X @ np.array([cos_s, sin_s]).T
        c2_s = X @ np.array([-sin_s, cos_s]).T

        rect = RectangleData()
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

    def _adoptive_range_segmentation(self, ox, oy):

        # Setup initial cluster
        S = []
        for i, _ in enumerate(ox):
            C = set()
            R = self.R0 + self.Rd * np.linalg.norm([ox[i], oy[i]])
            for j, _ in enumerate(ox):
                d = np.hypot(ox[i] - ox[j], oy[i] - oy[j])
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


class RectangleData:

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

    @staticmethod
    def calc_cross_point(a, b, c):
        x = (b[0] * -c[1] - b[1] * -c[0]) / (a[0] * b[1] - a[1] * b[0])
        y = (a[1] * -c[0] - a[0] * -c[1]) / (a[0] * b[1] - a[1] * b[0])
        return x, y


def main():

    # simulation parameters
    sim_time = 30.0  # simulation time
    dt = 0.2  # time tick

    angle_resolution = np.deg2rad(3.0)  # sensor angle resolution

    v1 = VehicleSimulator(-10.0, 0.0, np.deg2rad(90.0),
                          0.0, 50.0 / 3.6, 3.0, 5.0)
    v2 = VehicleSimulator(20.0, 10.0, np.deg2rad(180.0),
                          0.0, 50.0 / 3.6, 4.0, 10.0)

    l_shape_fitting = LShapeFitting()
    lidar_sim = LidarSimulator()

    time = 0.0
    while time <= sim_time:
        time += dt

        v1.update(dt, 0.1, 0.0)
        v2.update(dt, 0.1, -0.05)

        ox, oy = lidar_sim.get_observation_points([v1, v2], angle_resolution)

        rects, id_sets = l_shape_fitting.fitting(ox, oy)

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.axis("equal")
            plt.plot(0.0, 0.0, "*r")
            v1.plot()
            v2.plot()

            # Plot range observation
            for ids in id_sets:
                x = [ox[i] for i in range(len(ox)) if i in ids]
                y = [oy[i] for i in range(len(ox)) if i in ids]

                for (ix, iy) in zip(x, y):
                    plt.plot([0.0, ix], [0.0, iy], "-og")

                plt.plot([ox[i] for i in range(len(ox)) if i in ids],
                         [oy[i] for i in range(len(ox)) if i in ids],
                         "o")
            for rect in rects:
                rect.plot()

            plt.pause(0.1)

    print("Done")


if __name__ == '__main__':
    main()
