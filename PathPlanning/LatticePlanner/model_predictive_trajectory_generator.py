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
    x, y = [state.x], [state.y]

    for ikp in kp:
        state = update(state, v, ikp, dt, L)
        x.append(state.x)
        y.append(state.y)

    return x, y


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
    test_trajectory_generate()


if __name__ == '__main__':
    main()
