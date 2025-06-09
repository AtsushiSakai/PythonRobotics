"""

Path planning Sample Code with RRT with path smoothing

@author: AtsushiSakai(@Atsushi_twi)

"""

import math
import random
import matplotlib.pyplot as plt
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent))

from rrt import RRT

show_animation = True


def get_path_length(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.hypot(dx, dy)
        le += d

    return le


def get_target_point(path, targetL):
    le = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.hypot(dx, dy)
        le += d
        if le >= targetL:
            ti = i - 1
            lastPairLen = d
            break

    partRatio = (le - targetL) / lastPairLen

    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio

    return [x, y, ti]


def is_point_collision(x, y, obstacle_list, robot_radius):
    """
    Check whether a single point collides with any obstacle.

    This function calculates the Euclidean distance between the given point (x, y)
    and each obstacle center. If the distance is less than or equal to the sum of
    the obstacle's radius and the robot's radius, a collision is detected.

    Args:
        x (float): X-coordinate of the point to check.
        y (float): Y-coordinate of the point to check.
        obstacle_list (List[Tuple[float, float, float]]): List of obstacles defined as (ox, oy, radius).
        robot_radius (float): Radius of the robot, used to inflate the obstacles.

    Returns:
        bool: True if the point is in collision with any obstacle, False otherwise.
    """
    for (ox, oy, obstacle_radius) in obstacle_list:
        d = math.hypot(ox - x, oy - y)
        if d <= obstacle_radius + robot_radius:
            return True  # Collided
    return False


def line_collision_check(first, second, obstacle_list, robot_radius=0.0, sample_step=0.2):
    """
    Check if the line segment between `first` and `second` collides with any obstacle.
    Considers the robot_radius by inflating the obstacle size.

    Args:
        first (List[float]): Start point of the line [x, y]
        second (List[float]): End point of the line [x, y]
        obstacle_list (List[Tuple[float, float, float]]): Obstacles as (x, y, radius)
        robot_radius (float): Radius of robot
        sample_step (float): Distance between sampling points along the segment

    Returns:
        bool: True if collision-free, False otherwise
    """
    x1, y1 = first[0], first[1]
    x2, y2 = second[0], second[1]

    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)

    if length == 0:
        # Degenerate case: point collision check
        return not is_point_collision(x1, y1, obstacle_list, robot_radius)

    steps = int(length / sample_step) + 1  # Sampling every sample_step along the segment

    for i in range(steps + 1):
        t = i / steps
        x = x1 + t * dx
        y = y1 + t * dy

        if is_point_collision(x, y, obstacle_list, robot_radius):
            return False  # Collision found

    return True  # Safe


def path_smoothing(path, max_iter, obstacle_list, robot_radius=0.0):
    """
    Smooths a given path by iteratively replacing segments with shortcut connections,
    while ensuring the new segments are collision-free.

    The algorithm randomly picks two points along the original path and attempts to
    connect them with a straight line. If the line does not collide with any obstacles
    (considering the robot's radius), the intermediate path points between them are
    replaced with the direct connection.

    Args:
        path (List[List[float]]): The original path as a list of [x, y] coordinates.
        max_iter (int): Number of iterations for smoothing attempts.
        obstacle_list (List[Tuple[float, float, float]]): List of obstacles represented as
            (x, y, radius).
        robot_radius (float, optional): Radius of the robot, used to inflate obstacle size
            during collision checking. Defaults to 0.0.

    Returns:
        List[List[float]]: The smoothed path as a list of [x, y] coordinates.

    Example:
        >>> smoothed = path_smoothing(path, 1000, obstacle_list, robot_radius=0.5)
    """
    le = get_path_length(path)

    for i in range(max_iter):
        # Sample two points
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        first = get_target_point(path, pickPoints[0])
        second = get_target_point(path, pickPoints[1])

        if first[2] <= 0 or second[2] <= 0:
            continue

        if (second[2] + 1) > len(path):
            continue

        if second[2] == first[2]:
            continue

        # collision check
        if not line_collision_check(first, second, obstacle_list, robot_radius):
            continue

        # Create New path
        newPath = []
        newPath.extend(path[:first[2] + 1])
        newPath.append([first[0], first[1]])
        newPath.append([second[0], second[1]])
        newPath.extend(path[second[2] + 1:])
        path = newPath
        le = get_path_length(path)

    return path


def main():
    # ====Search Path with RRT====
    # Parameter
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,radius]
    rrt = RRT(start=[0, 0], goal=[6, 10],
              rand_area=[-2, 15], obstacle_list=obstacleList,
              robot_radius=0.3)
    path = rrt.planning(animation=show_animation)

    # Path smoothing
    maxIter = 1000
    smoothedPath = path_smoothing(path, maxIter, obstacleList,
                                  robot_radius=rrt.robot_radius)

    # Draw final path
    if show_animation:
        rrt.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')

        plt.plot([x for (x, y) in smoothedPath], [
            y for (x, y) in smoothedPath], '-c')

        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()


if __name__ == '__main__':
    main()
