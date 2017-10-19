#! /usr/bin/python
# -*- coding: utf-8 -*-
"""

Reeds Shepp path planner sample code

author Atsushi Sakai(@Atsushi_twi)

License MIT

"""
import reeds_shepp
import math


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    u"""
    Plot arrow
    """
    import matplotlib.pyplot as plt

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def reeds_shepp_path_planning(start_x, start_y, start_yaw,
                              end_x, end_y, end_yaw, curvature):
    step_size = 0.1
    q0 = [start_x, start_y, start_yaw]
    q1 = [end_x, end_y, end_yaw]
    qs = reeds_shepp.path_sample(q0, q1, 1.0 / curvature, step_size)
    xs = [q[0] for q in qs]
    ys = [q[1] for q in qs]
    yaw = [q[2] for q in qs]

    xs.append(end_x)
    ys.append(end_y)
    yaw.append(end_yaw)

    clen = reeds_shepp.path_length(q0, q1, 1.0 / curvature)
    pathtypeTuple = reeds_shepp.path_type(q0, q1, 1.0 / curvature)

    ptype = ""
    for t in pathtypeTuple:
        if t == 1:
            ptype += "L"
        elif t == 2:
            ptype += "S"
        elif t == 3:
            ptype += "R"

    return xs, ys, yaw, ptype, clen


if __name__ == '__main__':
    print("Reeds Shepp path planner sample start!!")
    import matplotlib.pyplot as plt

    start_x = 1.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = math.radians(0.0)  # [rad]

    end_x = -0.0  # [m]
    end_y = -3.0  # [m]
    end_yaw = math.radians(-45.0)  # [rad]

    curvature = 1.0

    px, py, pyaw, mode, clen = reeds_shepp_path_planning(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)

    plt.plot(px, py, label="final course " + str(mode))

    # plotting
    plot_arrow(start_x, start_y, start_yaw)
    plot_arrow(end_x, end_y, end_yaw)

    for (ix, iy, iyaw) in zip(px, py, pyaw):
        plot_arrow(ix, iy, iyaw, fc="b")
    #  print(clen)

    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()
