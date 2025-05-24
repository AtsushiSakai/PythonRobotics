"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

"""
import numpy as np
import math
import matplotlib.pyplot as plt

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
from utils.angle import angle_mod

# Parameters
k = 0.1  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2.9  # [m] wheel base of vehicle


# Vehicle parameters
LENGTH = WB + 1.0  # Vehicle length
WIDTH = 2.0  # Vehicle width
WHEEL_LEN = 0.6  # Wheel length
WHEEL_WIDTH = 0.2  # Wheel width
MAX_STEER = math.pi / 4  # Maximum steering angle [rad]


show_animation = True
pause_simulation = False  # Flag for pause simulation
is_reverse_mode = False   # Flag for reverse driving mode

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, is_reverse=False):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direction = -1 if is_reverse else 1  # Direction based on reverse flag
        self.rear_x = self.x - self.direction * ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - self.direction * ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.direction * self.v / WB * math.tan(delta) * dt
        self.yaw = angle_mod(self.yaw)
        self.v += a * dt
        self.rear_x = self.x - self.direction * ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - self.direction * ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.direction = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.direction.append(state.direction)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    # Reverse steering angle when reversing
    delta = state.direction * math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    # Limit steering angle to max value
    delta = np.clip(delta, -MAX_STEER, MAX_STEER)

    return delta, ind


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

def plot_vehicle(x, y, yaw, steer=0.0, color='blue', is_reverse=False):
    """
    Plot vehicle model with four wheels
    Args:
        x, y: Vehicle center position
        yaw: Vehicle heading angle
        steer: Steering angle
        color: Vehicle color
        is_reverse: Flag for reverse mode
    """
    # Adjust heading angle in reverse mode
    if is_reverse:
        yaw = angle_mod(yaw + math.pi)  # Rotate heading by 180 degrees
        steer = -steer  # Reverse steering direction

    def plot_wheel(x, y, yaw, steer=0.0, color=color):
        """Plot single wheel"""
        wheel = np.array([
            [-WHEEL_LEN/2, WHEEL_WIDTH/2],
            [WHEEL_LEN/2, WHEEL_WIDTH/2],
            [WHEEL_LEN/2, -WHEEL_WIDTH/2],
            [-WHEEL_LEN/2, -WHEEL_WIDTH/2],
            [-WHEEL_LEN/2, WHEEL_WIDTH/2]
        ])

        # Rotate wheel if steering
        if steer != 0:
            c, s = np.cos(steer), np.sin(steer)
            rot_steer = np.array([[c, -s], [s, c]])
            wheel = wheel @ rot_steer.T

        # Apply vehicle heading rotation
        c, s = np.cos(yaw), np.sin(yaw)
        rot_yaw = np.array([[c, -s], [s, c]])
        wheel = wheel @ rot_yaw.T

        # Translate to position
        wheel[:, 0] += x
        wheel[:, 1] += y

        # Plot wheel with color
        plt.plot(wheel[:, 0], wheel[:, 1], color=color)

    # Calculate vehicle body corners
    corners = np.array([
        [-LENGTH/2, WIDTH/2],
        [LENGTH/2, WIDTH/2],
        [LENGTH/2, -WIDTH/2],
        [-LENGTH/2, -WIDTH/2],
        [-LENGTH/2, WIDTH/2]
    ])

    # Rotation matrix
    c, s = np.cos(yaw), np.sin(yaw)
    Rot = np.array([[c, -s], [s, c]])

    # Rotate and translate vehicle body
    rotated = corners @ Rot.T
    rotated[:, 0] += x
    rotated[:, 1] += y

    # Plot vehicle body
    plt.plot(rotated[:, 0], rotated[:, 1], color=color)

    # Plot wheels (darker color for front wheels)
    front_color = 'darkblue'
    rear_color = color
    
    # Plot four wheels
    # Front left
    plot_wheel(x + LENGTH/4 * c - WIDTH/2 * s,
              y + LENGTH/4 * s + WIDTH/2 * c,
              yaw, steer, front_color)
    # Front right  
    plot_wheel(x + LENGTH/4 * c + WIDTH/2 * s,
              y + LENGTH/4 * s - WIDTH/2 * c,
              yaw, steer, front_color)
    # Rear left
    plot_wheel(x - LENGTH/4 * c - WIDTH/2 * s,
              y - LENGTH/4 * s + WIDTH/2 * c,
              yaw, color=rear_color)
    # Rear right
    plot_wheel(x - LENGTH/4 * c + WIDTH/2 * s,
              y - LENGTH/4 * s - WIDTH/2 * c,
              yaw, color=rear_color)

    # Add direction arrow
    arrow_length = LENGTH/3
    plt.arrow(x, y,
             -arrow_length * math.cos(yaw) if is_reverse else arrow_length * math.cos(yaw),
             -arrow_length * math.sin(yaw) if is_reverse else arrow_length * math.sin(yaw),
             head_width=WIDTH/4, head_length=WIDTH/4,
             fc='r', ec='r', alpha=0.5)

# Keyboard event handler
def on_key(event):
    global pause_simulation
    if event.key == ' ':  # Space key
        pause_simulation = not pause_simulation
    elif event.key == 'escape':
        exit(0)

def main():
    #  target course
    cx = -1 * np.arange(0, 50, 0.5) if is_reverse_mode else np.arange(0, 50, 0.5)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    target_speed = 10.0 / 3.6  # [m/s]

    T = 100.0  # max simulation time

    # initial state
    state = State(x=-0.0, y=-3.0, yaw=math.pi if is_reverse_mode else 0.0, v=0.0, is_reverse=is_reverse_mode)

    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)

    while T >= time and lastIndex > target_ind:

        # Calc control input
        ai = proportional_control(target_speed, state.v)
        di, target_ind = pure_pursuit_steer_control(
            state, target_course, target_ind)

        state.update(ai, di)  # Control vehicle

        time += dt
        states.append(time, state)
        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event', on_key)
            # Pass is_reverse parameter
            plot_vehicle(state.x, state.y, state.yaw, di, is_reverse=is_reverse_mode)
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.legend()  # Add legend display

            # Add pause state display
            if pause_simulation:
                plt.text(0.02, 0.95, 'PAUSED', transform=plt.gca().transAxes,
                        bbox=dict(facecolor='red', alpha=0.5))

            plt.pause(0.001)

            # Handle pause state
            while pause_simulation:
                plt.pause(0.1)  # Reduce CPU usage
                if not plt.get_fignums():  # Check if window is closed
                    exit(0)

    # Test
    assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:  # pragma: no cover
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
