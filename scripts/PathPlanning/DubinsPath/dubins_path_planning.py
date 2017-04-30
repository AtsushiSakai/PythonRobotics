#! /usr/bin/python
# -*- coding: utf-8 -*-
"""

Dubins path planner sample code

author Atsushi Sakai(@Atsushi_twi)

Liscense MIT

"""
import math


def pi_2_pi(angle):
    """
    """

    while(angle >= math.pi):
        angle = angle - 2.0 * math.pi

    while(angle <= -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def fmodr(x, y):
    return x - y * math.floor(x / y)


def mod2pi(theta):
    return fmodr(theta, 2 * math.pi)


def LSL(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp0 = d + sa - sb
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb))
    if p_squared < 0:
        return 0
    tmp1 = math.atan2((cb - ca), tmp0)
    t = mod2pi(-alpha + tmp1)
    p = math.sqrt(p_squared)
    q = mod2pi(beta - tmp1)
    #  print(math.degrees(t), p, math.degrees(q))

    return t, p, q


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

    # nomalize
    dx = ex - sx
    dy = ey - sy
    D = math.sqrt(dx ** 2.0 + dy ** 2.0)
    d = D / c
    #  print(dx, dy, D, d)

    theta = mod2pi(math.atan2(dy, dx))
    alpha = mod2pi(syaw - theta)
    beta = mod2pi(eyaw - theta)
    print(theta, alpha, beta, d)

    t, p, q = LSL(alpha, beta, d)

    px, py, pyaw = generate_course(t, p, q)

    return px, py, pyaw


def generate_course(t, p, q):

    px = [0.0]
    py = [0.0]
    pyaw = [0.0]

    d = 0.001

    pd = 0.0
    while pd <= abs(t):

        px.append(px[-1] + d * math.cos(pyaw[-1]))
        py.append(py[-1] + d * math.sin(pyaw[-1]))
        pyaw.append(pyaw[-1] + d * 1.0)

        pd += d

    pd = 0.0
    while pd <= abs(p):
        px.append(px[-1] + d * math.cos(pyaw[-1]))
        py.append(py[-1] + d * math.sin(pyaw[-1]))
        pyaw.append(pyaw[-1])

        pd += d

    pd = 0.0
    while pd <= abs(q):
        px.append(px[-1] + d * math.cos(pyaw[-1]))
        py.append(py[-1] + d * math.sin(pyaw[-1]))
        pyaw.append(pyaw[-1] + d * 1.0)
        pd += d

    return px, py, pyaw


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

    end_x = -1.0  # [m]
    end_y = 10.0  # [m]
    end_yaw = math.radians(135.0)  # [rad]

    curvature = 1.0

    px, py, pyaw = dubins_path_planning(start_x, start_y, start_yaw,
                                        end_x, end_y, end_yaw, curvature)

    #  print(px, py)
    plt.plot(px, py, "-r")
    # plotting
    __plot_arrow(start_x, start_y, start_yaw)
    __plot_arrow(end_x, end_y, end_yaw)

    plt.grid(True)
    plt.axis("equal")
    plt.show()
