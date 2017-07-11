"""
Model trajectory generator

author: Atsushi Sakai
"""

import numpy as np
import scipy.interpolate
import matplotlib.pyplot as plt
import math
#  import unicycle_model

L = 1.0
ds = 0.1


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def generate_trajectory(s, km, kf, k0):

    n = s / ds
    v = 10.0 / 3.6  # [m/s]
    time = s / v  # [s]
    tk = np.array([0.0, time / 2.0, time])
    kk = np.array([k0, km, kf])
    t = np.arange(0.0, time, time / n)
    kp = scipy.interpolate.spline(tk, kk, t, order=2)
    dt = time / n

    #  plt.plot(t, kp)
    #  plt.show()

    state = State()
    x, y, yaw = [state.x], [state.y], [state.yaw]

    for ikp in kp:
        state = update(state, v, ikp, dt, L)
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)

    return x, y, yaw


def update(state, v, delta, dt, L):

    state.v = v
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.yaw = pi_2_pi(state.yaw)

    return state


def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    u"""
    Plot arrow
    """
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              fc=fc, ec=ec, head_width=width, head_length=width)
    plt.plot(x, y)
    plt.plot(0, 0)


def calc_diff(target, x, y, yaw):
    d = np.array([x[-1] - target.x, y[-1] - target.y, yaw[-1] - target.yaw])
    return d


def calc_J(target, p, h, k0):
    xp, yp, yawp = generate_trajectory(p[0, 0] + h[0, 0], p[1, 0], p[2, 0], k0)
    dp = calc_diff(target, xp, yp, yawp)
    d1 = np.matrix(dp / h[0, 0]).T

    xp, yp, yawp = generate_trajectory(p[0, 0], p[1, 0] + h[1, 0], p[2, 0], k0)
    dp = calc_diff(target, xp, yp, yawp)
    #  xn, yn, yawn = generate_trajectory(p[0, 0], p[1, 0] - h[1, 0], p[2, 0], k0)
    #  dn = calc_diff(target, xn, yn, yawn)
    #  d2 = np.matrix((dp - dn) / 2.0 * h[1, 0]).T
    d2 = np.matrix(dp / h[1, 0]).T

    xp, yp, yawp = generate_trajectory(p[0, 0], p[1, 0], p[2, 0] + h[2, 0], k0)
    dp = calc_diff(target, xp, yp, yawp)
    #  xn, yn, yawn = generate_trajectory(p[0, 0], p[1, 0], p[2, 0] - h[2, 0], k0)
    #  dn = calc_diff(target, xn, yn, yawn)
    #  d3 = np.matrix((dp - dn) / 2.0 * h[2, 0]).T
    d3 = np.matrix(dp / h[2, 0]).T
    #  print(d1, d2, d3)

    J = np.hstack((d1, d2, d3))
    #  print(J)

    return J


def optimize_trajectory(target, k0):

    p = np.matrix([5.0, 0.0, 0.0]).T
    h = np.matrix([0.1, 0.003, 0.003]).T

    for i in range(1000):
        xc, yc, yawc = generate_trajectory(p[0], p[1], p[2], k0)
        dc = np.matrix(calc_diff(target, xc, yc, yawc)).T

        if np.linalg.norm(dc) <= 0.1:
            break

        J = calc_J(target, p, h, k0)

        dp = - np.linalg.inv(J) * dc

        p += np.array(dp)

        #  print(p)
        #  plt.clf()
        #  plot_arrow(target.x, target.y, target.yaw)
        #  plt.plot(xc, yc, "-r")
        #  plt.axis("equal")
        #  plt.grid(True)
        #  #  plt.show()
        #  plt.pause(0.1)

    plot_arrow(target.x, target.y, target.yaw)
    plt.plot(xc, yc, "-r")
    plt.axis("equal")
    plt.grid(True)

    print("done")


def test_optimize_trajectory():

    target = State(x=5.0, y=2.0, yaw=math.radians(00.0))
    k0 = 0.0
    #  s = 5.0  # [m]
    #  km = math.radians(30.0)
    #  kf = math.radians(-30.0)

    optimize_trajectory(target, k0)

    #  x, y = generate_trajectory(s, km, kf, k0)

    #  plt.plot(x, y, "-r")
    plot_arrow(target.x, target.y, target.yaw)
    plt.axis("equal")
    plt.grid(True)
    plt.show()


def test_trajectory_generate():
    s = 5.0  # [m]
    k0 = 0.0
    km = math.radians(30.0)
    kf = math.radians(-30.0)

    #  plt.plot(xk, yk, "xr")
    #  plt.plot(t, kp)
    #  plt.show()

    x, y = generate_trajectory(s, km, kf, k0)

    plt.plot(x, y, "-r")
    plt.axis("equal")
    plt.grid(True)
    plt.show()


def main():
    print(__file__ + " start!!")
    #  test_trajectory_generate()
    test_optimize_trajectory()


if __name__ == '__main__':
    main()
