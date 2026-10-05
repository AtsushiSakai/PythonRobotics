"""Forward path tracking with Vector Pursuit and a kinematic bicycle model.

Author: OpenAI Codex
Reference: J. Wit, C. D. Crane III, D. Armstrong, Autonomous Ground Vehicle
Path Tracking, Journal of Robotic Systems 21(8), 439-449, 2004.
https://doi.org/10.1002/rob.20031

Positions are at the rear axle in meters, yaw/steering are radians, and
velocity is m/s. This example tracks a smooth, non-self-intersecting path;
it does not plan around obstacles or drive in reverse.
"""

from dataclasses import dataclass, replace
import math
import pathlib
import sys

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
from PathPlanning.CubicSpline import cubic_spline_planner
from utils.angle import angle_mod

WHEELBASE = 2.9  # [m]
MAX_STEER = math.radians(45.0)
LOOK_AHEAD = 4.0  # [m] measured along the sampled path
TIME_RATIO = 2.0  # k = rotation correction time / translation time
TARGET_SPEED = 2.0  # [m/s]
SPEED_GAIN = 1.0  # [1/s]
GOAL_TOLERANCE = 0.3  # [m]
DT = 0.05  # [s]
MAX_TIME = 80.0  # [s]
PATH_SPACING = 0.1  # [m] spline parameter increment
show_animation = True


@dataclass
class State:
    """Rear-axle position (m), yaw (rad), and forward speed (m/s)."""

    x: float = 0.0
    y: float = -2.0
    yaw: float = 0.0
    speed: float = 0.0

    def update(self, acceleration, steering):
        """Apply acceleration (m/s²) and bounded steering (rad) for DT seconds."""
        steering = float(np.clip(steering, -MAX_STEER, MAX_STEER))
        self.x += self.speed * math.cos(self.yaw) * DT
        self.y += self.speed * math.sin(self.yaw) * DT
        self.yaw = angle_mod(self.yaw + self.speed / WHEELBASE * math.tan(steering) * DT)
        self.speed += acceleration * DT


def vector_pursuit_curvature(target_x, target_y, heading_error, time_ratio=TIME_RATIO):
    """Return signed curvature (1/m) for a target in the rear-axle frame.

    The target must be strictly ahead (target_x > 0, meters). heading_error
    is target yaw minus vehicle yaw (radians); time_ratio is the positive
    dimensionless k in the paper. Positive curvature turns left.

    The signed translation arc angle is beta = 2*atan2(target_y, target_x).
    Its length is d / sinc(beta/2). Combining translation and rotation
    screws gives ((k - 1)*beta + heading_error) / (k * arc_length).
    This form includes the straight-ahead limit without dividing by beta.
    """
    if time_ratio <= 0.0:
        raise ValueError("time_ratio must be positive")
    if target_x <= 0.0:
        raise ValueError("Vector Pursuit requires a target strictly ahead")
    heading_error = angle_mod(heading_error)
    chord = math.hypot(target_x, target_y)
    arc_angle = 2.0 * math.atan2(target_y, target_x)
    # NumPy's sinc uses pi*x, whereas the geometric expression uses sin(x)/x.
    arc_length = chord / np.sinc(arc_angle / (2.0 * math.pi))
    return ((time_ratio - 1.0) * arc_angle + heading_error) / (time_ratio * arc_length)


def steering_control(state, target, time_ratio=TIME_RATIO):
    """Return bounded bicycle steering toward a world-frame [x, y, yaw] target."""
    dx, dy = target[0] - state.x, target[1] - state.y
    cosine, sine = math.cos(state.yaw), math.sin(state.yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    curvature = vector_pursuit_curvature(local_x, local_y,
                                         target[2] - state.yaw, time_ratio)
    return float(np.clip(math.atan(WHEELBASE * curvature), -MAX_STEER, MAX_STEER))


def search_target_index(state, course, previous_index=0):
    """Find a forward path sample LOOK_AHEAD meters beyond the nearest sample.

    course contains rows [x (m), y (m), tangent yaw (rad)] ordered along the
    path. The returned nearest index never precedes previous_index. Target
    selection follows sampled arc length and is clamped to the final sample.
    """
    distances = np.hypot(course[previous_index:, 0] - state.x,
                         course[previous_index:, 1] - state.y)
    nearest_index = previous_index + int(np.argmin(distances))
    target_index = nearest_index
    distance = 0.0
    while target_index < len(course) - 1 and distance < LOOK_AHEAD:
        distance += np.linalg.norm(course[target_index + 1, :2] - course[target_index, :2])
        target_index += 1
    return target_index, nearest_index


def create_course():
    """Sample an S-shaped cubic-spline path with tangent headings."""
    waypoint_x = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    waypoint_y = [0.0, 0.0, 8.0, -8.0, 6.0, 0.0, 0.0]
    x, y, yaw, _, _ = cubic_spline_planner.calc_spline_course(
        waypoint_x, waypoint_y, ds=PATH_SPACING)
    return np.column_stack((x, y, yaw))


def simulate(course=None, initial_state=None, max_time=MAX_TIME):
    """Track a course and return it with state, target, and steering histories.

    History rows are [time (s), x (m), y (m), yaw (rad), speed (m/s)].
    The initial state is copied. Reaching the final look-ahead sample does
    not end the run; the rear axle must be within GOAL_TOLERANCE of the goal.
    A timeout raises RuntimeError instead of reporting successful tracking.
    """
    if course is None:
        course = create_course()
    state = State() if initial_state is None else replace(initial_state)
    history = [[0.0, state.x, state.y, state.yaw, state.speed]]
    targets, steering_history = [], []
    nearest_index = 0
    for step in range(int(max_time / DT) + 1):
        goal_distance = math.hypot(course[-1, 0] - state.x, course[-1, 1] - state.y)
        if goal_distance <= GOAL_TOLERANCE:
            return course, np.array(history), np.array(targets), np.array(steering_history)
        if step == int(max_time / DT):
            break
        target_index, nearest_index = search_target_index(state, course, nearest_index)
        steering = steering_control(state, course[target_index])
        target_speed = min(TARGET_SPEED, SPEED_GAIN * goal_distance)
        acceleration = SPEED_GAIN * (target_speed - state.speed)
        targets.append(target_index)
        steering_history.append(steering)
        state.update(acceleration, steering)
        history.append([(step + 1) * DT, state.x, state.y, state.yaw, state.speed])
    raise RuntimeError("Vector Pursuit did not reach the goal within max_time")


def create_animation(course, history, targets, steering_history):
    """Animate a completed simulation; the returned object can be saved as a GIF."""
    fig, (ax, steer_ax) = plt.subplots(2, 1, figsize=(9, 6),
                                     gridspec_kw={"height_ratios": [3, 1]})
    ax.plot(course[:, 0], course[:, 1], "k--", label="Reference path")
    ax.plot(*course[0, :2], "go", label="Path start")
    ax.plot(*course[-1, :2], "r*", markersize=12, label="Goal")
    track, = ax.plot([], [], "b-", label="Rear-axle trajectory")
    body, = ax.plot([], [], "b-", linewidth=4, label="Vehicle heading")
    target, = ax.plot([], [], "mo", label="Look-ahead target")
    target_heading, = ax.plot([], [], "m-", linewidth=2)
    ax.set(xlabel="x [m]", ylabel="y [m]", aspect="equal")
    ax.legend(loc="upper left", ncol=3, fontsize=8)
    ax.grid(True)
    steering_line, = steer_ax.plot([], [], "b-")
    steer_ax.axhline(math.degrees(MAX_STEER), color="r", linestyle="--")
    steer_ax.axhline(-math.degrees(MAX_STEER), color="r", linestyle="--")
    steer_ax.set(xlabel="Time [s]", ylabel="Steering [deg]",
                 xlim=(0.0, max(DT, history[-1, 0])), ylim=(-50.0, 50.0))
    steer_ax.grid(True)
    fig.tight_layout()

    def update(index):
        time, x, y, yaw, speed = history[index]
        track.set_data(history[:index + 1, 1], history[:index + 1, 2])
        body.set_data([x, x + WHEELBASE * math.cos(yaw)],
                      [y, y + WHEELBASE * math.sin(yaw)])
        if len(targets):
            tx, ty, target_yaw = course[targets[min(index, len(targets) - 1)]]
            target.set_data([tx], [ty])
            target_heading.set_data([tx, tx + WHEELBASE * math.cos(target_yaw)],
                                    [ty, ty + WHEELBASE * math.sin(target_yaw)])
        steering_line.set_data(history[:index, 0], np.rad2deg(steering_history[:index]))
        ax.set_title(f"Vector Pursuit: t = {time:.1f} s, speed = {speed:.1f} m/s")
        return track, body, target, target_heading, steering_line

    frames = list(range(0, len(history) - 1, 4)) + [len(history) - 1]
    return FuncAnimation(fig, update, frames=frames, interval=1000 * 4 * DT, repeat=False)


def main():
    """Run the forward Vector Pursuit example, optionally displaying its animation."""
    result = simulate()
    if show_animation:
        animation = create_animation(*result)
        plt.show()
        return animation
    return None


if __name__ == "__main__":
    main()
