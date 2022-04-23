"""

Move to specified pose (with Robot class)

Author: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai (@Atsushi_twi)
        Seied Muhammad Yazdian (@Muhammad-Yazdian)

P.I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7

"""

import matplotlib.pyplot as plt
import numpy as np
import copy
from move_to_pose import PathFinderController

# Simulation parameters
TIME_DURATION = 1000
TIME_STEP = 0.01
AT_TARGET_ACCEPTANCE_THRESHOLD = 0.01
SHOW_ANIMATION = True
PLOT_WINDOW_SIZE_X = 20
PLOT_WINDOW_SIZE_Y = 20
PLOT_FONT_SIZE = 8

simulation_running = True
all_robots_are_at_target = False


class Pose:
    """2D pose"""

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


class Robot:
    """
    Constructs an instantiate of the 3-DOF wheeled Robot navigating on a
    2D plane

    Parameters
    ----------
    name : (string)
        The name of the robot
    color : (string)
        The color of the robot
    max_linear_speed : (float)
        The maximum linear speed that the robot can go
    max_angular_speed : (float)
        The maximum angular speed that the robot can rotate about its vertical
        axis
    path_finder_controller : (PathFinderController)
        A configurable controller to finds the path and calculates command
        linear and angular velocities.
    """

    def __init__(self, name, color, max_linear_speed, max_angular_speed,
                 path_finder_controller):
        self.name = name
        self.color = color
        self.MAX_LINEAR_SPEED = max_linear_speed
        self.MAX_ANGULAR_SPEED = max_angular_speed
        self.path_finder_controller = path_finder_controller
        self.x_traj = []
        self.y_traj = []
        self.pose = Pose(0, 0, 0)
        self.pose_start = Pose(0, 0, 0)
        self.pose_target = Pose(0, 0, 0)
        self.is_at_target = False

    def set_start_target_poses(self, pose_start, pose_target):
        """
        Sets the start and target positions of the robot

        Parameters
        ----------
        pose_start : (Pose)
            Start postion of the robot (see the Pose class)
        pose_target : (Pose)
            Target postion of the robot (see the Pose class)
        """
        self.pose_start = copy.copy(pose_start)
        self.pose_target = pose_target
        self.pose = pose_start

    def move(self, dt):
        """
        Moves the robot for one time step increment

        Parameters
        ----------
        dt : (float)
            time step
        """
        self.x_traj.append(self.pose.x)
        self.y_traj.append(self.pose.y)

        rho, linear_velocity, angular_velocity = \
            self.path_finder_controller.calc_control_command(
                self.pose_target.x - self.pose.x,
                self.pose_target.y - self.pose.y,
                self.pose.theta, self.pose_target.theta)

        if rho < AT_TARGET_ACCEPTANCE_THRESHOLD:
            self.is_at_target = True

        if abs(linear_velocity) > self.MAX_LINEAR_SPEED:
            linear_velocity = (np.sign(linear_velocity)
                               * self.MAX_LINEAR_SPEED)

        if abs(angular_velocity) > self.MAX_ANGULAR_SPEED:
            angular_velocity = (np.sign(angular_velocity)
                                * self.MAX_ANGULAR_SPEED)

        self.pose.theta = self.pose.theta + angular_velocity * dt
        self.pose.x = self.pose.x + linear_velocity * \
            np.cos(self.pose.theta) * dt
        self.pose.y = self.pose.y + linear_velocity * \
            np.sin(self.pose.theta) * dt


def run_simulation(robots):
    """Simulates all robots simultaneously"""
    global all_robots_are_at_target
    global simulation_running

    robot_names = []
    for instance in robots:
        robot_names.append(instance.name)

    time = 0
    while simulation_running and time < TIME_DURATION:
        time += TIME_STEP
        robots_are_at_target = []

        for instance in robots:
            if not instance.is_at_target:
                instance.move(TIME_STEP)
            robots_are_at_target.append(instance.is_at_target)

        if all(robots_are_at_target):
            simulation_running = False

        if SHOW_ANIMATION:
            plt.cla()
            plt.xlim(0, PLOT_WINDOW_SIZE_X)
            plt.ylim(0, PLOT_WINDOW_SIZE_Y)

            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            plt.text(0.3, PLOT_WINDOW_SIZE_Y - 1,
                     'Time: {:.2f}'.format(time),
                     fontsize=PLOT_FONT_SIZE)

            plt.text(0.3, PLOT_WINDOW_SIZE_Y - 2,
                     'Reached target: {} = '.format(robot_names)
                     + str(robots_are_at_target),
                     fontsize=PLOT_FONT_SIZE)

            for instance in robots:
                plt.arrow(instance.pose_start.x,
                          instance.pose_start.y,
                          np.cos(instance.pose_start.theta),
                          np.sin(instance.pose_start.theta),
                          color='r',
                          width=0.1)
                plt.arrow(instance.pose_target.x,
                          instance.pose_target.y,
                          np.cos(instance.pose_target.theta),
                          np.sin(instance.pose_target.theta),
                          color='g',
                          width=0.1)
                plot_vehicle(instance.pose.x,
                             instance.pose.y,
                             instance.pose.theta,
                             instance.x_traj,
                             instance.y_traj, instance.color)

            plt.pause(TIME_STEP)


def plot_vehicle(x, y, theta, x_traj, y_traj, color):
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = T @ p1_i
    p2 = T @ p2_i
    p3 = T @ p3_i

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color+'-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], color+'-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], color+'-')

    plt.plot(x_traj, y_traj, color+'--')


def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])


def main():
    pose_target = Pose(15, 15, -1)

    pose_start_1 = Pose(5, 2, 0)
    pose_start_2 = Pose(5, 2, 0)
    pose_start_3 = Pose(5, 2, 0)

    controller_1 = PathFinderController(5, 8, 2)
    controller_2 = PathFinderController(5, 16, 4)
    controller_3 = PathFinderController(10, 25, 6)

    robot_1 = Robot("Yellow Robot", "y", 12, 5, controller_1)
    robot_2 = Robot("Black Robot", "k", 16, 5, controller_2)
    robot_3 = Robot("Blue Robot", "b", 20, 5, controller_3)

    robot_1.set_start_target_poses(pose_start_1, pose_target)
    robot_2.set_start_target_poses(pose_start_2, pose_target)
    robot_3.set_start_target_poses(pose_start_3, pose_target)

    robots: list[Robot] = [robot_1, robot_2, robot_3]

    run_simulation(robots)


if __name__ == '__main__':
    main()
