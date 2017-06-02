#! /usr/bin/python
# -*- coding: utf-8 -*-
u"""
author: Atsushi Sakai
"""
import numpy as np
import math
import matplotlib.pyplot as plt
import unicycle_model


def main():
    cx = np.arange(0, 50, 0.1)
    cy = [math.sin(ix / 5.0) * 5.0 for ix in cx]

    T = 10.0
    dt = 0.1
    time = 0.0

    state = unicycle_model.State()

    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]

    while T >= time:
        ai = 1.0
        di = 0.01
        state = unicycle_model.update(state, ai, di)

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)

        time = time + dt

    plt.plot(cx, cy, ".r")
    plt.plot(x, y, "-b")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

    pass


if __name__ == '__main__':
    main()
