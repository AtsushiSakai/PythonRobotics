"""

Hybrid A* path planning

author: Atsushi Sakai (@Atsushi_twi)

"""

import sys
sys.path.append("../ReedsSheppPath/")

#  import random
import math
#  import numpy as np
import matplotlib.pyplot as plt
import reeds_shepp_path_planning

show_animation = True


def hybrid_a_star_planning(start, goal, ox, oy, xyreso, yawreso):

    rx, ry, ryaw = [], [], []

    return rx, ry, ryaw


def main():
    print("Start rrt start planning")
    # ====Search Path with RRT====

    ox, oy = [], []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)

    # Set Initial parameters
    start = [10.0, 10.0, math.radians(90.0)]
    goal = [50.0, 50.0, math.radians(-90.0)]

    xyreso = 2.0
    yawreso = math.radians(15.0)

    rx, ry, ryaw = hybrid_a_star_planning(
        start, goal, ox, oy, xyreso, yawreso)

    plt.plot(ox, oy, ".k")
    reeds_shepp_path_planning.plot_arrow(
        start[0], start[1], start[2])
    reeds_shepp_path_planning.plot_arrow(
        goal[0], goal[1], goal[2])

    plt.grid(True)
    plt.axis("equal")
    plt.show()

    print(__file__ + " start!!")


if __name__ == '__main__':
    main()
