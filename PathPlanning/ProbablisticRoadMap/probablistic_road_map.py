"""

Probablistic Road Map (PRM) Planner

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import matplotlib.pyplot as plt
from matplotrecorder import matplotrecorder
from pyfastnns import pyfastnns
matplotrecorder.donothing = True
import random

N_SAMPLE = 500
N_KNN = 20
MAX_EDGE_LEN = 30.0


def PRM_planning(sx, sy, gx, gy, ox, oy, rr):

    sample_x, sample_y = sample_points(sx, sy, gx, gy, rr, ox, oy)

    plt.plot(sample_x, sample_y, ".r")

    return [], []


def sample_points(sx, sy, gx, gy, rr, ox, oy):
    maxx = max(ox)
    maxy = max(oy)
    minx = min(ox)
    miny = min(oy)

    sample_x, sample_y = [], []

    nns = pyfastnns.NNS(np.vstack((ox, oy)).T)

    while len(sample_x) <= N_SAMPLE:
        tx = (random.random() - minx) * (maxx - minx)
        ty = (random.random() - miny) * (maxy - miny)

        index, dist = nns.search(np.matrix([tx, ty]).T)

        if dist[0] >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    robot_size = 5.0  # [m]

    ox = []
    oy = []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)

    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "xr")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")

    rx, ry = PRM_planning(sx, sy, gx, gy, ox, oy, robot_size)

    #  plt.plot(rx, ry, "-r")

    for i in range(20):
        matplotrecorder.save_frame()
    plt.show()

    matplotrecorder.save_movie("animation.gif", 0.1)


if __name__ == '__main__':
    main()
