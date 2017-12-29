"""

Path Planning with 4 point Beizer curve

author: Atsushi Sakai(@Atsushi_twi)

"""

import scipy.misc as scm
import numpy as np
import matplotlib.pyplot as plt
import math


def calc_4point_bezier_path(sx, sy, syaw, ex, ey, eyaw, offset):
    D = math.sqrt((sx - ex)**2 + (sy - ey)**2) / offset
    cp = np.array(
        [[sx, sy],
         [sx + D * math.cos(syaw), sy + D * math.sin(syaw)],
         [ex - D * math.cos(eyaw), ey - D * math.sin(eyaw)],
         [ex, ey]])

    traj = []
    for t in np.linspace(0, 1, 100):
        traj.append(bezier(3, t, cp))
    P = np.array(traj)

    return P, cp


def bernstein(n, i, t):
    return scm.comb(n, i) * t**i * (1 - t)**(n - i)


def bezier(n, t, q):
    p = np.zeros(2)
    for i in range(n + 1):
        p += bernstein(n, i, t) * q[i]
    return p


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    u"""
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def main():
    start_x = 10.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = math.radians(180.0)  # [rad]

    end_x = -0.0  # [m]
    end_y = -3.0  # [m]
    end_yaw = math.radians(-45.0)  # [rad]
    offset = 3.0

    P, cp = calc_4point_bezier_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)

    plt.plot(P.T[0], P.T[1], label="Bezier Path")
    plt.plot(cp.T[0], cp.T[1], '--o', label="Control Points")
    plot_arrow(start_x, start_y, start_yaw)
    plot_arrow(end_x, end_y, end_yaw)
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.show()


def main2():
    start_x = 10.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = math.radians(180.0)  # [rad]

    end_x = -0.0  # [m]
    end_y = -3.0  # [m]
    end_yaw = math.radians(-45.0)  # [rad]
    offset = 3.0

    for offset in np.arange(1.0, 5.0, 1.0):
        P, cp = calc_4point_bezier_path(
            start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)
        plt.plot(P.T[0], P.T[1], label="Offset=" + str(offset))
    plot_arrow(start_x, start_y, start_yaw)
    plot_arrow(end_x, end_y, end_yaw)
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
    #  main2()
