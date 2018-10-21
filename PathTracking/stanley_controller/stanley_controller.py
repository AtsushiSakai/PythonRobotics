"""
Path tracking simulation with Stanley steering control and PID speed control.

DOI: 10.1002/rob.20147

author: Atsushi Sakai (@Atsushi_twi)
"""
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.append("../../PathPlanning/CubicSpline/")

import cubic_spline_planner
from utils_math import normalize_angle
from utils_unit import kph2mps, mps2kph


k = 0.5  # control gain
Kp = 1.0  # speed propotional gain
dt = 0.1  # [s] time difference
L = 2.9  # [m] Wheel base of vehicle
max_steer = np.deg2rad(30.0)  # [rad] max steering angle

show_animation = True



class State:
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.v += acceleration * dt

        self.yaw = normalize_angle(self.yaw)


def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    squared_errors = [(fx - icx)**2 + (fy - icy)**2
        for (icx, icy) in zip(cx, cy)]
    min_squared_errors = min(squared_errors)
    target_idx = squared_errors.index(min_squared_errors)
    error_of_front_axle = np.sqrt(min_squared_errors)

    target_yaw = normalize_angle(np.arctan2(
        fy - cy[target_idx], fx - cx[target_idx]) - state.yaw)

    if target_yaw > 0.0:
        error_of_front_axle *= -1

    return target_idx, error_of_front_axle


def main():
    """Plot an example of Stanley steering control on a cubic spline."""
    #  target course
    ax = [0.0, 100.0, 100.0, 50.0, 60.0]
    ay = [0.0, 0.0, -30.0, -20.0, 0.0]

    cx, cy, cyaw, _, _ = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

    target_speed = kph2mps(30.0)  # [m/s]

    max_simulation_time = 100.0

    # Initial state
    state = State(x=-0.0, y=5.0, yaw=np.deg2rad(20.0), v=0.0)

    last_idx = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)

    while max_simulation_time >= time and last_idx > target_idx:
        ai = pid_control(target_speed, state.v)
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        state.update(ai, di)

        time += dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.legend(loc='best')
            plt.title("Speed[km/h]:{:.2f}".format(mps2kph(state.v)))
            plt.pause(0.001)

    # Test
    assert last_idx >= target_idx, "Cannot reach goal"

    if show_animation:
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend(loc='best')
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(t, mps2kph(np.array(v)), "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()
