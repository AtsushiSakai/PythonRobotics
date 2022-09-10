"""

Model trajectory generator

author: Atsushi Sakai(@Atsushi_twi)

"""

import math
import matplotlib.pyplot as plt
import numpy as np
import sys
import pathlib
path_planning_dir = pathlib.Path(__file__).parent.parent
sys.path.append(str(path_planning_dir))

import ModelPredictiveTrajectoryGenerator.motion_model as motion_model

# optimization parameter
max_iter = 100
h = np.array([0.5, 0.02, 0.02]).T  # parameter sampling distance
cost_th = 0.1

show_animation = True


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
    """
    Plot arrow
    """
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              fc=fc, ec=ec, head_width=width, head_length=width)
    plt.plot(x, y)
    plt.plot(0, 0)


def calc_diff(target, x, y, yaw):
    d = np.array([target.x - x[-1],
                  target.y - y[-1],
                  motion_model.pi_2_pi(target.yaw - yaw[-1])])

    return d


def calc_j(target, p, h, k0):
    xp, yp, yawp = motion_model.generate_last_state(
        p[0, 0] + h[0], p[1, 0], p[2, 0], k0)
    dp = calc_diff(target, [xp], [yp], [yawp])
    xn, yn, yawn = motion_model.generate_last_state(
        p[0, 0] - h[0], p[1, 0], p[2, 0], k0)
    dn = calc_diff(target, [xn], [yn], [yawn])
    d1 = np.array((dp - dn) / (2.0 * h[0])).reshape(3, 1)

    xp, yp, yawp = motion_model.generate_last_state(
        p[0, 0], p[1, 0] + h[1], p[2, 0], k0)
    dp = calc_diff(target, [xp], [yp], [yawp])
    xn, yn, yawn = motion_model.generate_last_state(
        p[0, 0], p[1, 0] - h[1], p[2, 0], k0)
    dn = calc_diff(target, [xn], [yn], [yawn])
    d2 = np.array((dp - dn) / (2.0 * h[1])).reshape(3, 1)

    xp, yp, yawp = motion_model.generate_last_state(
        p[0, 0], p[1, 0], p[2, 0] + h[2], k0)
    dp = calc_diff(target, [xp], [yp], [yawp])
    xn, yn, yawn = motion_model.generate_last_state(
        p[0, 0], p[1, 0], p[2, 0] - h[2], k0)
    dn = calc_diff(target, [xn], [yn], [yawn])
    d3 = np.array((dp - dn) / (2.0 * h[2])).reshape(3, 1)

    J = np.hstack((d1, d2, d3))

    return J


def selection_learning_param(dp, p, k0, target):
    mincost = float("inf")
    mina = 1.0
    maxa = 2.0
    da = 0.5

    for a in np.arange(mina, maxa, da):
        tp = p + a * dp
        xc, yc, yawc = motion_model.generate_last_state(
            tp[0], tp[1], tp[2], k0)
        dc = calc_diff(target, [xc], [yc], [yawc])
        cost = np.linalg.norm(dc)

        if cost <= mincost and a != 0.0:
            mina = a
            mincost = cost

    #  print(mincost, mina)
    #  input()

    return mina


def show_trajectory(target, xc, yc):  # pragma: no cover
    plt.clf()
    plot_arrow(target.x, target.y, target.yaw)
    plt.plot(xc, yc, "-r")
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.1)


def optimize_trajectory(target, k0, p):
    for i in range(max_iter):
        xc, yc, yawc = motion_model.generate_trajectory(p[0], p[1], p[2], k0)
        dc = np.array(calc_diff(target, xc, yc, yawc)).reshape(3, 1)

        cost = np.linalg.norm(dc)
        if cost <= cost_th:
            print("path is ok cost is:" + str(cost))
            break

        J = calc_j(target, p, h, k0)
        try:
            dp = - np.linalg.inv(J) @ dc
        except np.linalg.linalg.LinAlgError:
            print("cannot calc path LinAlgError")
            xc, yc, yawc, p = None, None, None, None
            break
        alpha = selection_learning_param(dp, p, k0, target)

        p += alpha * np.array(dp)
        #  print(p.T)

        if show_animation:  # pragma: no cover
            show_trajectory(target, xc, yc)
    else:
        xc, yc, yawc, p = None, None, None, None
        print("cannot calc path")

    return xc, yc, yawc, p


def test_optimize_trajectory():  # pragma: no cover

    #  target = motion_model.State(x=5.0, y=2.0, yaw=np.deg2rad(00.0))
    target = motion_model.State(x=5.0, y=2.0, yaw=np.deg2rad(90.0))
    k0 = 0.0

    init_p = np.array([6.0, 0.0, 0.0]).reshape(3, 1)

    x, y, yaw, p = optimize_trajectory(target, k0, init_p)

    if show_animation:
        show_trajectory(target, x, y)
        plot_arrow(target.x, target.y, target.yaw)
        plt.axis("equal")
        plt.grid(True)
        plt.show()


def main():  # pragma: no cover
    print(__file__ + " start!!")
    test_optimize_trajectory()


if __name__ == '__main__':
    main()
