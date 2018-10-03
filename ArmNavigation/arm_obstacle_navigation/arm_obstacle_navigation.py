"""
Obstacle navigation using A* on a toroidal grid

Author: Daniel Ingram (daniel-s-ingram)
"""
from math import pi
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import from_levels_and_colors

plt.ion()

# Simulation parameters
M = 100
obstacles = [[1.75, 0.75, 0.6], [0.55, 1.5, 0.5], [0, -1, 0.25]]


def main():
    arm = NLinkArm([1, 1], [0, 0])
    start = (10, 50)
    goal = (58, 56)
    grid = get_occupancy_grid(arm, obstacles)
    plt.imshow(grid)
    plt.show()
    route = astar_torus(grid, start, goal)
    for node in route:
        theta1 = 2 * pi * node[0] / M - pi
        theta2 = 2 * pi * node[1] / M - pi
        arm.update_joints([theta1, theta2])
        arm.plot(obstacles=obstacles)


def detect_collision(line_seg, circle):
    """
    Determines whether a line segment (arm link) is in contact
    with a circle (obstacle).
    Credit to: http://doswa.com/2009/07/13/circle-segment-intersectioncollision.html
    Args:
        line_seg: List of coordinates of line segment endpoints e.g. [[1, 1], [2, 2]]
        circle: List of circle coordinates and radius e.g. [0, 0, 0.5] is a circle centered
                at the origin with radius 0.5

    Returns:
        True if the line segment is in contact with the circle
        False otherwise
    """
    a_vec = np.array([line_seg[0][0], line_seg[0][1]])
    b_vec = np.array([line_seg[1][0], line_seg[1][1]])
    c_vec = np.array([circle[0], circle[1]])
    radius = circle[2]
    line_vec = b_vec - a_vec
    line_mag = np.linalg.norm(line_vec)
    circle_vec = c_vec - a_vec
    proj = circle_vec.dot(line_vec / line_mag)
    if proj <= 0:
        closest_point = a_vec
    elif proj >= line_mag:
        closest_point = b_vec
    else:
        closest_point = a_vec + line_vec * proj / line_mag
    if np.linalg.norm(closest_point - c_vec) > radius:
        return False
    else:
        return True


def get_occupancy_grid(arm, obstacles):
    """
    Discretizes joint space into M values from -pi to +pi
    and determines whether a given coordinate in joint space
    would result in a collision between a robot arm and obstacles
    in its environment.

    Args:
        arm: An instance of NLinkArm
        obstacles: A list of obstacles, with each obstacle defined as a list
                   of xy coordinates and a radius. 

    Returns:
        Occupancy grid in joint space
    """
    grid = [[0 for _ in range(M)] for _ in range(M)]
    theta_list = [2 * i * pi / M for i in range(-M // 2, M // 2 + 1)]
    for i in range(M):
        for j in range(M):
            arm.update_joints([theta_list[i], theta_list[j]])
            points = arm.points
            collision_detected = False
            for k in range(len(points) - 1):
                for obstacle in obstacles:
                    line_seg = [points[k], points[k + 1]]
                    collision_detected = detect_collision(line_seg, obstacle)
                    if collision_detected:
                        break
                if collision_detected:
                    break
            grid[i][j] = int(collision_detected)
    return np.array(grid)


def astar_torus(grid, start_node, goal_node):
    """
    Finds a path between an initial and goal joint configuration using
    the A* Algorithm on a tororiadal grid.

    Args:
        grid: An occupancy grid (ndarray)
        start_node: Initial joint configuation (tuple)
        goal_node: Goal joint configuration (tuple)

    Returns:
        Obstacle-free route in joint space from start_node to goal_node
    """
    colors = ['white', 'black', 'red', 'pink', 'yellow', 'green', 'orange']
    levels = [0, 1, 2, 3, 4, 5, 6, 7]
    cmap, norm = from_levels_and_colors(levels, colors)

    grid[start_node] = 4
    grid[goal_node] = 5

    parent_map = [[() for _ in range(M)] for _ in range(M)]

    X, Y = np.meshgrid([i for i in range(M)], [i for i in range(M)])
    heuristic_map = np.abs(X - goal_node[1]) + np.abs(Y - goal_node[0])
    for i in range(heuristic_map.shape[0]):
        for j in range(heuristic_map.shape[1]):
            heuristic_map[i, j] = min(heuristic_map[i, j],
                                      i + 1 + heuristic_map[M - 1, j],
                                      M - i + heuristic_map[0, j],
                                      j + 1 + heuristic_map[i, M - 1],
                                      M - j + heuristic_map[i, 0]
                                      )

    explored_heuristic_map = np.full((M, M), np.inf)
    distance_map = np.full((M, M), np.inf)
    explored_heuristic_map[start_node] = heuristic_map[start_node]
    distance_map[start_node] = 0
    while True:
        grid[start_node] = 4
        grid[goal_node] = 5

        current_node = np.unravel_index(
            np.argmin(explored_heuristic_map, axis=None), explored_heuristic_map.shape)
        min_distance = np.min(explored_heuristic_map)
        if (current_node == goal_node) or np.isinf(min_distance):
            break

        grid[current_node] = 2
        explored_heuristic_map[current_node] = np.inf

        i, j = current_node[0], current_node[1]

        neighbors = []
        if i - 1 >= 0:
            neighbors.append((i - 1, j))
        else:
            neighbors.append((M - 1, j))

        if i + 1 < M:
            neighbors.append((i + 1, j))
        else:
            neighbors.append((0, j))

        if j - 1 >= 0:
            neighbors.append((i, j - 1))
        else:
            neighbors.append((i, M - 1))

        if j + 1 < M:
            neighbors.append((i, j + 1))
        else:
            neighbors.append((i, 0))

        for neighbor in neighbors:
            if grid[neighbor] == 0 or grid[neighbor] == 5:
                distance_map[neighbor] = distance_map[current_node] + 1
                explored_heuristic_map[neighbor] = heuristic_map[neighbor]
                parent_map[neighbor[0]][neighbor[1]] = current_node
                grid[neighbor] = 3
        '''
        plt.cla()
        plt.imshow(grid, cmap=cmap, norm=norm, interpolation=None)
        plt.show()
        plt.pause(1e-5)
        '''

    if np.isinf(explored_heuristic_map[goal_node]):
        route = []
        print("No route found.")
    else:
        route = [goal_node]
        while parent_map[route[0][0]][route[0][1]] is not ():
            route.insert(0, parent_map[route[0][0]][route[0][1]])

        print("The route found covers %d grid cells." % len(route))
        for i in range(1, len(route)):
            grid[route[i]] = 6
            plt.cla()
            plt.imshow(grid, cmap=cmap, norm=norm, interpolation=None)
            plt.show()
            plt.pause(1e-2)

    return route


class NLinkArm(object):
    """
    Class for controlling and plotting a planar arm with an arbitrary number of links.
    """

    def __init__(self, link_lengths, joint_angles):
        self.n_links = len(link_lengths)
        if self.n_links is not len(joint_angles):
            raise ValueError()

        self.link_lengths = np.array(link_lengths)
        self.joint_angles = np.array(joint_angles)
        self.points = [[0, 0] for _ in range(self.n_links + 1)]

        self.lim = sum(link_lengths)
        self.update_points()

    def update_joints(self, joint_angles):
        self.joint_angles = joint_angles
        self.update_points()

    def update_points(self):
        for i in range(1, self.n_links + 1):
            self.points[i][0] = self.points[i - 1][0] + \
                self.link_lengths[i - 1] * \
                np.cos(np.sum(self.joint_angles[:i]))
            self.points[i][1] = self.points[i - 1][1] + \
                self.link_lengths[i - 1] * \
                np.sin(np.sum(self.joint_angles[:i]))

        self.end_effector = np.array(self.points[self.n_links]).T

    def plot(self, obstacles=[]):
        plt.cla()

        for obstacle in obstacles:
            circle = plt.Circle(
                (obstacle[0], obstacle[1]), radius=0.5 * obstacle[2], fc='k')
            plt.gca().add_patch(circle)

        for i in range(self.n_links + 1):
            if i is not self.n_links:
                plt.plot([self.points[i][0], self.points[i + 1][0]],
                         [self.points[i][1], self.points[i + 1][1]], 'r-')
            plt.plot(self.points[i][0], self.points[i][1], 'k.')

        plt.xlim([-self.lim, self.lim])
        plt.ylim([-self.lim, self.lim])
        plt.draw()
        plt.pause(1e-5)


if __name__ == '__main__':
    main()