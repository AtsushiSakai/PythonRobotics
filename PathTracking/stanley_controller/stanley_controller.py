"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

"""
import math
import matplotlib.pyplot as plt

from pycubicspline import pycubicspline


k = 0.5  # control gain
Kp = 1.0  # speed propotional gain
dt = 0.1  # [s] time difference
L = 2.9  # [m] Wheel base of vehicle
max_steer = math.radians(30.0)  # [rad] max steering angle

show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):

    if delta >= max_steer:
        delta = max_steer
    elif delta <= -max_steer:
        delta = -max_steer

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.yaw = pi_2_pi(state.yaw)
    state.v = state.v + a * dt

    return state


def PIDControl(target, current):
    a = Kp * (target - current)

    return a


def stanley_control(state, cx, cy, cyaw, pind):

    ind, efa = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    theta_e = pi_2_pi(cyaw[ind] - state.yaw)
    theta_d = math.atan2(k * efa, state.v)
    delta = theta_e + theta_d

    return delta, ind


def pi_2_pi(angle):
    while (angle > math.pi):
        angle = angle - 2.0 * math.pi

    while (angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def calc_target_index(state, cx, cy):

    # calc frant axle position
    fx = state.x + L * math.cos(state.yaw)
    fy = state.y + L * math.sin(state.yaw)

    # search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    mind = min(d)
    ind = d.index(mind)

    tyaw = pi_2_pi(math.atan2(fy - cy[ind], fx - cx[ind]) - state.yaw)
    if tyaw > 0.0:
        mind = - mind

    return ind, mind


def main():
    #  target course
    ax = [0.0, 100.0, 100.0, 50.0, 60.0]
    ay = [0.0, 0.0, -30.0, -20.0, 0.0]

    cx, cy, cyaw, ck, s = pycubicspline.calc_spline_course(ax, ay, ds=0.1)

    target_speed = 30.0 / 3.6  # [m/s]

    T = 100.0  # max simulation time

    # initial state
    state = State(x=-0.0, y=5.0, yaw=math.radians(20.0), v=0.0)

    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind, mind = calc_target_index(state, cx, cy)

    while T >= time and lastIndex > target_ind:
        ai = PIDControl(target_speed, state.v)
        di, target_ind = stanley_control(state, cx, cy, cyaw, target_ind)
        state = update(state, ai, di)

        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:
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
    main()
