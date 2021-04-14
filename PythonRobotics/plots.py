#!/usr/bin/env python
"""
plotting functions

the functions are kept in a separate file to increase the
readability of main algorithm.
"""

import math
import matplotlib.pyplot as plt


def plot_arrow(x, y, yaw, length=0.1, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def plot_point(xy, txt=None, style='o'):
    plt.plot(xy[0], xy[1],  style)
    if txt is not None:
        plt.text(xy[0]+0.1, xy[1], txt)


def plot_path(points, style='o-'):
    x = [pt[0] for pt in points]
    y = [pt[1] for pt in points]
    plt.plot(x, y, style)


def plot_vector(p1, p2):
    plt.quiver(p1.x, p1.y, (p2-p1).x, (p2-p1).y,
               angles='xy', scale_units='xy', scale=1)
