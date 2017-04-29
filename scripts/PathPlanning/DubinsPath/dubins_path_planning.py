#! /usr/bin/python
# -*- coding: utf-8 -*-
"""

Dubins path planner sample code

author Atsushi Sakai(@Atsushi_twi)

Liscense MIT

"""
import math


def dubins_path_planning(sx, sy, syaw, ex, ey, eyaw, c):
    """
    Dubins path plannner

    input:
        sx x position of start point [m]
        sy y position of start point [m]
        syaw yaw angle of start point [rad]
        ex x position of end point [m]
        ey y position of end point [m]
        eyaw yaw angle of end point [rad]
        c curvature [1/m]

    output:
        path [x,y...]

    """

    path = []

    return path


def __plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    u"""
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            __plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


if __name__ == '__main__':
    print("Dubins path planner sample start!!")
    import matplotlib.pyplot as plt

    start_x = 0.0  # [m]
    start_y = 0.0  # [m]
    start_yaw = math.radians(0.0)  # [rad]

    end_x = 10.0  # [m]
    end_y = 10.0  # [m]
    end_yaw = math.radians(45.0)  # [rad]

    curvature = 1.0

    # plotting
    __plot_arrow(start_x, start_y, start_yaw)
    __plot_arrow(end_x, end_y, end_yaw)

    plt.grid(True)
    plt.axis("equal")
    plt.show()
