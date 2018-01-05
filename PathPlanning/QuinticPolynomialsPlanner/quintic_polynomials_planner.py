"""

Quinitc Polynomials Planner

author: Atsushi Sakai (@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import math


def quinic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga):

    return None


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
    print(__file__ + " start!!")

    sx = 10.0
    sy = 10.0
    syaw = math.radians(10.0)
    sv = 0.0  # [m/s]
    sa = 0.0
    gx = 30.0
    gy = 20.0
    gyaw = math.radians(90.0)
    gv = 0.0  # [m/s]
    ga = 0.0

    quinic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga)

    plot_arrow(sx, sy, syaw)
    plot_arrow(gx, gy, gyaw)
    plt.grid(True)
    plt.axis("equal")

    plt.show()


if __name__ == '__main__':
    main()
