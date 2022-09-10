"""

Path tracking simulation with pure pursuit steering control and PID speed control.

author: Atsushi Sakai

"""
import math

import matplotlib.pyplot as plt
import numpy as np

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from ClosedLoopRRTStar import unicycle_model

Kp = 2.0  # speed propotional gain
Lf = 0.5  # look-ahead distance
T = 100.0  # max simulation time
goal_dis = 0.5
stop_speed = 0.5

#  animation = True
animation = False


def PIDControl(target, current):
    a = Kp * (target - current)

    if a > unicycle_model.accel_max:
        a = unicycle_model.accel_max
    elif a < -unicycle_model.accel_max:
        a = -unicycle_model.accel_max

    return a


def pure_pursuit_control(state, cx, cy, pind):

    ind, dis = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    #  print(parent_index, ind)
    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v <= 0.0:  # back
        alpha = math.pi - alpha

    delta = math.atan2(2.0 * unicycle_model.L * math.sin(alpha) / Lf, 1.0)

    if delta > unicycle_model.steer_max:
        delta = unicycle_model.steer_max
    elif delta < - unicycle_model.steer_max:
        delta = -unicycle_model.steer_max

    return delta, ind, dis


def calc_target_index(state, cx, cy):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = np.hypot(dx, dy)
    mindis = min(d)

    ind = np.argmin(d)

    L = 0.0

    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.hypot(dx, dy)
        ind += 1

    #  print(mindis)
    return ind, mindis


def closed_loop_prediction(cx, cy, cyaw, speed_profile, goal):

    state = unicycle_model.State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)

    #  lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    a = [0.0]
    d = [0.0]
    target_ind, mindis = calc_target_index(state, cx, cy)
    find_goal = False

    maxdis = 0.5

    while T >= time:
        di, target_ind, dis = pure_pursuit_control(state, cx, cy, target_ind)

        target_speed = speed_profile[target_ind]
        target_speed = target_speed * \
            (maxdis - min(dis, maxdis - 0.1)) / maxdis

        ai = PIDControl(target_speed, state.v)
        state = unicycle_model.update(state, ai, di)

        if abs(state.v) <= stop_speed and target_ind <= len(cx) - 2:
            target_ind += 1

        time = time + unicycle_model.dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.hypot(dx, dy) <= goal_dis:
            find_goal = True
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        a.append(ai)
        d.append(di)

        if target_ind % 1 == 0 and animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed:" + str(round(state.v, 2))
                      + "tind:" + str(target_ind))
            plt.pause(0.0001)

    else:
        print("Time out!!")

    return t, x, y, yaw, v, a, d, find_goal


def set_stop_point(target_speed, cx, cy, cyaw):
    speed_profile = [target_speed] * len(cx)
    forward = True

    d = []
    is_back = False

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]
        d.append(math.hypot(dx, dy))
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
            #  print(i_yaw, move_direction, dx, dy)
        elif not is_back and not forward:
            speed_profile[i] = 0.0
            forward = True
            #  plt.plot(cx[i], cy[i], "xb")
            #  print(i_yaw, move_direction, dx, dy)
    speed_profile[0] = 0.0
    if is_back:
        speed_profile[-1] = -stop_speed
    else:
        speed_profile[-1] = stop_speed

    d.append(d[-1])

    return speed_profile, d


def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile, d = set_stop_point(target_speed, cx, cy, cyaw)

    if animation:  # pragma: no cover
        plt.plot(speed_profile, "xb")

    return speed_profile


def extend_path(cx, cy, cyaw):

    dl = 0.1
    dl_list = [dl] * (int(Lf / dl) + 1)

    move_direction = math.atan2(cy[-1] - cy[-3], cx[-1] - cx[-3])
    is_back = abs(move_direction - cyaw[-1]) >= math.pi / 2.0

    for idl in dl_list:
        if is_back:
            idl *= -1
        cx = np.append(cx, cx[-1] + idl * math.cos(cyaw[-1]))
        cy = np.append(cy, cy[-1] + idl * math.sin(cyaw[-1]))
        cyaw = np.append(cyaw, cyaw[-1])

    return cx, cy, cyaw


def main():  # pragma: no cover
    #  target course
    cx = np.arange(0, 50, 0.1)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    target_speed = 5.0 / 3.6

    T = 15.0  # max simulation time

    state = unicycle_model.State(x=-0.0, y=-3.0, yaw=0.0, v=0.0)
    #  state = unicycle_model.State(x=-1.0, y=-5.0, yaw=0.0, v=-30.0 / 3.6)
    #  state = unicycle_model.State(x=10.0, y=5.0, yaw=0.0, v=-30.0 / 3.6)
    #  state = unicycle_model.State(
    #  x=3.0, y=5.0, yaw=np.deg2rad(-40.0), v=-10.0 / 3.6)
    #  state = unicycle_model.State(
    #  x=3.0, y=5.0, yaw=np.deg2rad(40.0), v=50.0 / 3.6)

    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind, dis = calc_target_index(state, cx, cy)

    while T >= time and lastIndex > target_ind:
        ai = PIDControl(target_speed, state.v)
        di, target_ind, _ = pure_pursuit_control(state, cx, cy, target_ind)
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

    plt.subplots(1)
    plt.plot(cx, cy, ".r", label="course")
    plt.plot(x, y, "-b", label="trajectory")
    plt.legend()
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.grid(True)

    plt.subplots(1)
    plt.plot(t, [iv * 3.6 for iv in v], "-r")
    plt.xlabel("Time[s]")
    plt.ylabel("Speed[km/h]")
    plt.grid(True)
    plt.show()


if __name__ == '__main__':  # pragma: no cover
    print("Pure pursuit path tracking simulation start")
    main()
