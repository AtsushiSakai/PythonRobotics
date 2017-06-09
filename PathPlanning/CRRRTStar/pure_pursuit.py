#! /usr/bin/python
# -*- coding: utf-8 -*-
u"""

Path tracking simulation with pure pursuit steering control and PID speed control.

author: Atsushi Sakai

"""
import numpy as np
import math
import matplotlib.pyplot as plt
import unicycle_model

Kp = 1.0  # speed propotional gain
Lf = 1.0  # look-ahead distance


def PIDControl(target, current):
    a = Kp * (target - current)

    return a


def pure_pursuit_control(state, cx, cy, pind):

    if state.v >= 0:
        ind = calc_nearest_index(state, cx[pind:], cy[pind:])
    else:
        ind = calc_nearest_index(state, cx[:pind + 1], cy[:pind + 1])

    if state.v >= 0:
        ind = ind + pind

    tx = cx[ind]
    ty = cy[ind]

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        if alpha > 0:
            alpha = math.pi - alpha
        else:
            alpha = math.pi + alpha

    delta = math.atan2(2.0 * unicycle_model.L * math.sin(alpha) / Lf, 1.0)

    if state.v < 0:  # back
        delta = delta * -1.0

    return delta, ind


def calc_nearest_index(state, cx, cy):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [abs(math.sqrt(idx ** 2 + idy ** 2) -
             Lf) for (idx, idy) in zip(dx, dy)]

    ind = d.index(min(d))

    return ind


def main():
    # target course
    cx = np.arange(0, 50, 0.1)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
    target_speed = 30.0 / 3.6

    T = 15.0  # max simulation time

    state = unicycle_model.State(x=-1.0, y=-5.0, yaw=0.0, v=0.0)
    #  state = unicycle_model.State(x=-1.0, y=-5.0, yaw=0.0, v=-30.0 / 3.6)
    #  state = unicycle_model.State(x=10.0, y=5.0, yaw=0.0, v=-30.0 / 3.6)
    #  state = unicycle_model.State(
    #  x=3.0, y=5.0, yaw=math.radians(-40.0), v=-10.0 / 3.6)
    #  state = unicycle_model.State(
    #  x=3.0, y=5.0, yaw=math.radians(40.0), v=50.0 / 3.6)

    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_nearest_index(state, cx, cy)

    while T >= time and lastIndex > target_ind:
        ai = PIDControl(target_speed, state.v)
        di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
        state = unicycle_model.update(state, ai, di)

        time = time + unicycle_model.dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        #  plt.cla()
        #  plt.plot(cx, cy, ".r", label="course")
        #  plt.plot(x, y, "-b", label="trajectory")
        #  plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
        #  plt.axis("equal")
        #  plt.grid(True)
        #  plt.pause(0.1)
        #  input()

    flg, ax = plt.subplots(1)
    plt.plot(cx, cy, ".r", label="course")
    plt.plot(x, y, "-b", label="trajectory")
    plt.legend()
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.grid(True)

    flg, ax = plt.subplots(1)
    plt.plot(t, [iv * 3.6 for iv in v], "-r")
    plt.xlabel("Time[s]")
    plt.ylabel("Speed[km/h]")
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
