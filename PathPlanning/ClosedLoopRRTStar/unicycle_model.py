"""

Unicycle model class

author Atsushi Sakai

"""

import math
import numpy as np

dt = 0.05  # [s]
L = 0.9  # [m]
steer_max = np.deg2rad(40.0)
curvature_max = math.tan(steer_max) / L
curvature_max = 1.0 / curvature_max + 1.0

accel_max = 5.0


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.yaw = pi_2_pi(state.yaw)
    state.v = state.v + a * dt

    return state


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


if __name__ == '__main__':  # pragma: no cover
    print("start unicycle simulation")
    import matplotlib.pyplot as plt

    T = 100
    a = [1.0] * T
    delta = [np.deg2rad(1.0)] * T
    #  print(delta)
    #  print(a, delta)

    state = State()

    x = []
    y = []
    yaw = []
    v = []

    for (ai, di) in zip(a, delta):
        state = update(state, ai, di)

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)

    plt.subplots(1)
    plt.plot(x, y)
    plt.axis("equal")
    plt.grid(True)

    plt.subplots(1)
    plt.plot(v)
    plt.grid(True)

    plt.show()
