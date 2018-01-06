"""

Quinitc Polynomials Planner

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import matplotlib.pyplot as plt
import math


class quinic_polynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[T**3, T**4, T**5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        #  print(A)
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T**2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)
        #  print(x)
        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

        return xt


def quinic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga):

    vxs = sv * math.cos(syaw)
    vys = sv * math.sin(syaw)
    vxg = gv * math.cos(gyaw)
    vyg = gv * math.sin(gyaw)

    axs = sa * math.cos(syaw)
    ays = sa * math.sin(syaw)
    axg = ga * math.cos(gyaw)
    ayg = ga * math.sin(gyaw)

    T = 10.0
    dt = 0.1

    xqp = quinic_polynomial(sx, vxs, axs, gx, vxg, axg, T)
    yqp = quinic_polynomial(sy, vys, ays, gy, vyg, ayg, T)

    rx, ry = [], []
    for t in np.arange(0.0, T + dt, dt):
        rx.append(xqp.calc_point(t))
        ry.append(yqp.calc_point(t))

    return rx, ry


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
    sv = 10.0  # [m/s]
    sa = 0.0
    gx = 30.0
    gy = -10.0
    gyaw = math.radians(20.0)
    gv = 1.0  # [m/s]
    ga = 0.0

    rx, ry = quinic_polynomials_planner(
        sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga)

    plt.plot(rx, ry, "-r")
    plot_arrow(sx, sy, syaw)
    plot_arrow(gx, gy, gyaw)
    plt.grid(True)
    plt.axis("equal")

    plt.show()


if __name__ == '__main__':
    main()
