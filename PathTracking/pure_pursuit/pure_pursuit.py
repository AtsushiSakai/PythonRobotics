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
#  animation = True
animation = False


def PIDControl(target, current):
    a = Kp * (target - current)

    return a


def pure_pursuit_control(state, cx, cy, pind):

    ind = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    #  print(pind, ind)
    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = math.pi - alpha
        #  if alpha > 0:
        #  alpha = math.pi - alpha
        #  else:
        #  alpha = math.pi + alpha

    delta = math.atan2(2.0 * unicycle_model.L * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def calc_target_index(state, cx, cy):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]

    ind = d.index(min(d))

    L = 0.0

    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cx[ind + 1] - cx[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind


def closed_loop_prediction(cx, cy, cyaw, speed_profile, goal):

    T = 500.0  # max simulation time
    goal_dis = 0.3
    stop_speed = 0.05

    state = unicycle_model.State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)

    #  lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)

    while T >= time:
        di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
        ai = PIDControl(speed_profile[target_ind], state.v)
        state = unicycle_model.update(state, ai, di)

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time = time + unicycle_model.dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.sqrt(dx ** 2 + dy ** 2) <= goal_dis:
            print("Goal")
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if target_ind % 20 == 0 and animation:
            plt.cla()
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed:" + str(round(state.v, 2)) +
                      "tind:" + str(target_ind))
            plt.pause(0.0001)

    return t, x, y, yaw, v


def set_stop_point(target_speed, cx, cy, cyaw):
    speed_profile = [target_speed] * len(cx)
    forward = True

    d = []

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]
        d.append(math.sqrt(dx ** 2.0 + dy ** 2.0))
        iyaw = cyaw[i]
        move_direction = math.atan2(dy, dx)
        is_back = abs(move_direction - iyaw) >= math.pi / 2.0

        if dx == 0.0 and dy == 0.0:
            continue

        if is_back:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if is_back and forward:
            speed_profile[i] = 0.0
            forward = False
            #  plt.plot(cx[i], cy[i], "xb")
            #  print(iyaw, move_direction, dx, dy)
        elif not is_back and not forward:
            speed_profile[i] = 0.0
            forward = True
            #  plt.plot(cx[i], cy[i], "xb")
            #  print(iyaw, move_direction, dx, dy)
    speed_profile[0] = 0.0
    speed_profile[-1] = 0.0

    d.append(d[-1])

    return speed_profile, d


def calc_speed_profile(cx, cy, cyaw, target_speed, a):

    speed_profile, d = set_stop_point(target_speed, cx, cy, cyaw)

    nsp = len(speed_profile)

    #  plt.plot(speed_profile, "xb")

    # forward integration
    for i in range(nsp - 1):

        if speed_profile[i + 1] >= 0:  # forward
            tspeed = speed_profile[i] + a * d[i]
            if tspeed <= speed_profile[i + 1]:
                speed_profile[i + 1] = tspeed
        else:
            tspeed = speed_profile[i] - a * d[i]
            if tspeed >= speed_profile[i + 1]:
                speed_profile[i + 1] = tspeed

    #  plt.plot(speed_profile, "ok")

    # back integration
    for i in range(nsp - 1):
        if speed_profile[- i - 1] >= 0:  # forward
            tspeed = speed_profile[-i] + a * d[-i]
            if tspeed <= speed_profile[-i - 1]:
                speed_profile[-i - 1] = tspeed
        else:
            tspeed = speed_profile[-i] - a * d[-i]
            if tspeed >= speed_profile[-i - 1]:
                speed_profile[-i - 1] = tspeed

    #  flg, ax = plt.subplots(1)
    #  plt.plot(speed_profile, "-r")
    #  plt.show()

    return speed_profile


def extend_path(cx, cy, cyaw):

    dl = 0.1
    dl_list = [dl] * (int(Lf / dl) + 0)

    move_direction = math.atan2(cy[-1] - cy[-2], cx[-1] - cx[-2])
    is_back = abs(move_direction - cyaw[-1]) >= math.pi / 2.0

    for idl in dl_list:
        if is_back:
            idl *= -1
        cx = np.append(cx, cx[-1] + idl * math.cos(cyaw[-1]))
        cy = np.append(cy, cy[-1] + idl * math.sin(cyaw[-1]))
        cyaw = np.append(cyaw, cyaw[-1])

    return cx, cy, cyaw


def main():
    #  target course
    import numpy as np
    cx = np.arange(0, 50, 0.1)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    target_speed = 10.0 / 3.6

    T = 15.0  # max simulation time

    state = unicycle_model.State(x=-0.0, y=-3.0, yaw=0.0, v=0.0)
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
    target_ind = calc_target_index(state, cx, cy)

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


def main2():
    import pandas as pd
    data = pd.read_csv("rrt_course.csv")
    cx = np.array(data["x"])
    cy = np.array(data["y"])
    cyaw = np.array(data["yaw"])

    target_speed = 10.0 / 3.6
    a = 0.1

    goal = [cx[-1], cy[-1]]

    cx, cy, cyaw = extend_path(cx, cy, cyaw)

    speed_profile = calc_speed_profile(cx, cy, cyaw, target_speed, a)

    t, x, y, yaw, v = closed_loop_prediction(cx, cy, cyaw, speed_profile, goal)

    flg, ax = plt.subplots(1)
    plt.plot(cx, cy, ".r", label="course")
    plt.plot(x, y, "-b", label="trajectory")
    plt.plot(goal[0], goal[1], "xg", label="goal")
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
    #  main()
    main2()
