"""
Move to specified pose

Author: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai(@Atsushi_twi)

P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7
"""

import matplotlib.pyplot as plt
import numpy as np

# simulation parameters
Kp_rho = 9
Kp_alpha = 15
Kp_beta = -3
dt = 0.01

show_animation = True


def normalize_pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
    """
    rho is the distance between the robot and the goal position
    alpha is the angle to the goal relative to the heading of the robot
    beta is the angle between the robot's position and the goal position plus the goal angle

    Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards the goal
    Kp_beta*beta rotates the line so that it is parallel to the goal angle
    """
    x = x_start
    y = y_start
    theta = theta_start

    x_diff = x_goal - x
    y_diff = y_goal - y

    x_traj, y_traj = [], []

    rho = np.sqrt(x_diff**2 + y_diff**2)
    while rho > 0.001:
        x_traj.append(x)
        y_traj.append(y)

        x_diff = x_goal - x
        y_diff = y_goal - y

        """
        Restrict alpha and beta (angle differences) to the range
        [-pi, pi] to prevent unstable behavior e.g. difference going
        from 0 rad to 2*pi rad with slight turn
        """

        rho = np.hypot(x_diff, y_diff)
        alpha = normalize_pi_2_pi(np.arctan2(y_diff, x_diff) - theta)
        beta = normalize_pi_2_pi(theta_goal - theta - alpha)

        v = Kp_rho * rho
        w = Kp_alpha * alpha + Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v *= -1

        theta += w * dt
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt

        if show_animation:
            plt.cla()
            plt.arrow(x_start, y_start, np.cos(theta_start),
                      np.sin(theta_start), color='r', width=0.1)
            plt.arrow(x_goal, y_goal, np.cos(theta_goal),
                      np.sin(theta_goal), color='g', width=0.1)
            plot_vehicle(x, y, theta, x_traj, y_traj)


def plot_vehicle(x, y, theta, x_traj, y_traj):
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

    plt.plot(x_traj, y_traj, 'b--')

    plt.xlim(0, 20)
    plt.ylim(0, 20)

    plt.pause(dt)


def transformation_matrix(x, y, theta):
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    return np.array([
        [cos_theta, -sin_theta, x],
        [sin_theta, cos_theta, y],
        [0, 0, 1]
    ])


def main():

    for _ in range(5):
        x_start = 20 * np.random.random()
        y_start = 20 * np.random.random()
        theta_start = 2 * np.pi * np.random.random() - np.pi
        x_goal = 20 * np.random.random()
        y_goal = 20 * np.random.random()
        theta_goal = 2 * np.pi * np.random.random() - np.pi
        print("Initial x: {:.2f} m\nInitial y: {:.2f} m\nInitial theta: {:.2f} rad\n".format(
            x_start, y_start, theta_start))
        print("Goal x: {:.2f} m\nGoal y: {:.2f} m\nGoal theta: {:.2f} rad\n".format(
            x_goal, y_goal, theta_goal))
        move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)


if __name__ == '__main__':
    main()
