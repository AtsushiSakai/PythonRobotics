"""
Model trajectory generator

author: Atsushi Sakai
"""

import numpy as np
import scipy.interpolate
import matplotlib.pyplot as plt
import unicycle_model


def generate_trajectory(v, kp):

    state = State()


def main():
    print(__file__ + " start!!")

    v = 10.0 / 3.6  # [m/s]
    s = 10.0  # [m]
    k0 = 0.0
    km = -1.0
    kf = 1.0
    ntime = 20.0

    time = s / v  # [s]
    xk = np.array([0.0, time / 2.0, time])
    yk = np.array([k0, km, kf])
    t = np.append(np.arange(0.0, time, time / ntime), time)
    kp = scipy.interpolate.spline(xk, yk, t, order=2)

    #  plt.plot(xk, yk, "xr")
    #  plt.plot(t, kp)
    #  plt.show()

    x, y = generate_trajectory(v, kp)


if __name__ == '__main__':
    main()
