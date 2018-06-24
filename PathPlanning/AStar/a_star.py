"""
A* grid based planning
author: Atsushi Sakai(@Atsushi_twi)
Revised by Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
See also code of Christian Careaga (http://code.activestate.com/recipes/578919-python-a-pathfinding-with-binary-heap/)
"""

import matplotlib.pyplot as plt
import math
from operator import attrgetter

show_animation = True


class Node:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.gscore = 0
        self.fscore = 0

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.fscore)

    def __eq__(self, other):
        """
        useful Cell equivalence
        """
        if isinstance(other, self.__class__):
            return self.x == other.x and self.y == other.y
        else:
            return False


def calc_final_path(nstart, ngoal, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    current = ngoal
    while current != nstart:
        rx.append(current.x * reso)
        ry.append(current.y * reso)
        current = current.cameFrom

    return rx, ry


def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx / reso), round(sy / reso))
    ngoal = Node(round(gx / reso), round(gy / reso))
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = [nstart], []

    while openset:
        openset.sort(key=attrgetter("fscore"))
        # Remove the item with the smallest fscore value from the open set
        current = openset.pop(0)

        # show graph
        if show_animation:
            plt.plot(current.x * reso, current.y * reso, "xc")
            if len(closedset) % 10 == 0:
                plt.pause(0.001)

        if current == ngoal:
            print("Find goal")
            ngoal.cameFrom = current.cameFrom
            break

        # Add it to the closed set
        closedset.insert(0, current)

        # expand search grid based on motion model
        for i in range(len(motion)):
            neighbor = Node(current.x + motion[i][0], current.y + motion[i][1])

            # if tentative_g_score is eliminated we get the greedy algorithm instead
            tentative_g_score = current.gscore + heuristic(current, neighbor)

            if not verify_node(neighbor, obmap, minx, miny, maxx, maxy):
                continue

            if neighbor in closedset and tentative_g_score >= neighbor.gscore:
                continue

            if tentative_g_score < neighbor.gscore or neighbor not in openset:
                neighbor.cameFrom = current
                neighbor.gscore = tentative_g_score
                neighbor.fscore = tentative_g_score + heuristic(neighbor, ngoal)
                openset.append(neighbor)

    rx, ry = calc_final_path(nstart, ngoal, reso)

    return rx, ry


def heuristic(a, b):
    w = 10.0  # weight of heuristic
    d = w * math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[node.x][node.y]:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)

    # obstacle map generation
    obmap = [[False for i in range(xwidth)] for i in range(ywidth)]
    for ix in range(xwidth):
        x = ix + minx
        for iy in range(ywidth):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 1.0  # [m]
    robot_size = 1.0  # [m]

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

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()
