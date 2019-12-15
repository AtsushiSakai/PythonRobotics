"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """

    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_final_input(x, dw, config, goal, ob)

    return u, trajectory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yawrate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_dyawrate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.01  # [m/s]
        self.yawrate_reso = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 1.0  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 1.0  # [m] for collision check
        self.robot_length = 2.0  # [m] for collision check

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


def motion(x, u, dt):
    """
    motion model
    """

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def vectorized_motion(x, u, dt):
    """
    vectorized motion model
    """

    x[:, 2] += u[:, 1] * dt
    x[:, 0] += u[:, 0] * np.cos(x[:, 2]) * dt
    x[:, 1] += u[:, 0] * np.sin(x[:, 2]) * dt
    x[:, 3] = u[:, 0]
    x[:, 4] = u[:, 1]

    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin,vmax, yaw_rate min, yaw_rate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw

def predict_trajectories(x_init, samples, config):
    x = np.ones((samples.shape[0], x_init.shape[0])) * x_init
    trajectory = np.array(x)
    for _ in range(len(np.arange(0, config.predict_time+config.dt, config.dt))):
        x = vectorized_motion(x, np.array(samples), config.dt)
        trajectory = np.dstack([trajectory, x])
    trajectories = np.transpose(trajectory, [0, 2, 1])
    return trajectories


def calc_final_input(x, dw, config, goal, ob):
    """
    calculation final input with dinamic window
    """

    samples = np.dstack(np.meshgrid(np.arange(dw[2], dw[3], config.yawrate_reso),
                                    np.arange(dw[0], dw[1], config.v_reso)))
    samples = samples.reshape(-1, 2)
    samples = np.flip(samples)

    trajectories = predict_trajectories(x[:], samples, config)
    to_goal_costs = config.to_goal_cost_gain * calc_to_goal_cost(trajectories, goal)
    speed_costs = config.speed_cost_gain * (config.max_speed - trajectories[:, -1, 3])
    ob_costs = config.obstacle_cost_gain*calc_obstacle_cost(trajectories, ob, config)

    final_costs = to_goal_costs + speed_costs + ob_costs

    idx = np.argmin(final_costs)
    best_u = samples[idx].T
    best_trajectory = trajectories[idx]


    return best_u, best_trajectory


def calc_obstacle_cost(trajectories, ob, config):
    """
        calc obstacle cost inf: collision
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = (np.repeat(trajectories[:, :, 0], len(ox), axis=1) -
          np.tile(ox, (*trajectories.shape[0:2], 1)).reshape(trajectories.shape[0], -1))
    dy = (np.repeat(trajectories[:, :, 1], len(oy), axis=1) -
          np.tile(oy, (*trajectories.shape[0:2], 1)).reshape(trajectories.shape[0], -1))

    r = np.sqrt(np.square(dx) + np.square(dy))

    if config.robot_type == RobotType.rectangle:
        yaw = trajectories[:, :, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 3, 0, 1])
        local_obs = (np.tile(ob, (*trajectories.shape[0:2], 1)) -
                    np.repeat(trajectories[:, :, :2], len(oy), axis=1))

        a = np.tile(local_obs, (rot.shape[1], 1)).reshape(-1, 2)
        b = np.repeat(rot, local_obs.shape[1], axis=1).reshape(-1, 2, 2)
        out = np.einsum('...i,...ij', a, b)
        out = out.reshape(len(r), rot.shape[1] * local_obs.shape[1], 2)

        upper_check = out[:, :, 0] <= config.robot_length / 2
        right_check = out[:, :, 1] <= config.robot_width/2
        bottom_check = out[:, :, 0] >= -config.robot_length / 2
        left_check = out[:, :, 1] >= -config.robot_width/2
        check = np.logical_and(np.logical_and(upper_check, right_check),
                       np.logical_and(bottom_check, left_check))
    elif config.robot_type == RobotType.circle:
        check = r <= config.robot_radius
    check = np.sum(check, axis=1)
    minr = np.min(r, axis=1)
    scores = 1.0 / minr
    return np.where(check == 0, scores, float("Inf"))


def calc_to_goal_cost(trajectories, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectories[:, -1, 0]
    dy = goal[1] - trajectories[:, -1, 1]
    error_angle = np.arctan2(dy, dx)
    cost_angle = error_angle - trajectories[:, -1, 2]
    costs = np.abs(np.arctan2(np.sin(cost_angle), np.cos(cost_angle)))
    return costs


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")


def main(gx=10.0, gy=10.0, robot_type=RobotType.circle):
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])
    # obstacles [x(m) y(m), ....]
    ob = np.array([[-1, -1],
                   [0, 2],
                   [4.0, 2.0],
                   [5.0, 4.0],
                   [5.0, 5.0],
                   [5.0, 6.0],
                   [5.0, 9.0],
                   [8.0, 9.0],
                   [7.0, 9.0],
                   [8.0, 10.0],
                   [9.0, 11.0],
                   [12.0, 13.0],
                   [12.0, 12.0],
                   [15.0, 15.0],
                   [13.0, 13.0]
                   ])

    # input [forward speed, yaw_rate]
    config = Config()
    config.robot_type = robot_type
    trajectory = np.array(x)

    while True:
        u, predicted_trajectory = dwa_control(x, config, goal, ob)
        x = motion(x, u, config.dt)  # simulate robot
        trajectory = np.vstack((trajectory, x))  # store state history

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_robot(x[0], x[1], x[2], config)
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.pause(0.0001)

    plt.show()


if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)
