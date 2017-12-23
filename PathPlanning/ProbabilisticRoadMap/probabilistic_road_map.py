"""

Probablistic Road Map (PRM) Planner

author: Atsushi Sakai (@Atsushi_twi)

"""

import random
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotrecorder import matplotrecorder
from pyfastnns import pyfastnns
matplotrecorder.donothing = True

# parameter
N_SAMPLE = 500
N_KNN = 10
MAX_EDGE_LEN = 30.0  # [m] Maximum edge length


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def PRM_planning(sx, sy, gx, gy, ox, oy, rr):

    sample_x, sample_y = sample_points(sx, sy, gx, gy, rr, ox, oy)
    plt.plot(sample_x, sample_y, ".r")

    road_map = generate_roadmap(sample_x, sample_y, rr)

    rx, ry = dijkstra_planning(
        sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y)

    return rx, ry


def generate_roadmap(sample_x, sample_y, rr):

    road_map = []
    nsample = len(sample_x)
    skdtree = pyfastnns.NNS(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(nsample), sample_x, sample_y):

        index = skdtree.search(
            np.matrix([ix, iy]).T, k=nsample)
        edge_id = []

        for ii in range(1, len(index[0][0][0])):
            #  nx = sample_x[index[i]]
            #  ny = sample_y[index[i]]

            #  if !is_collision(ix, iy, nx, ny, rr, okdtree)
            edge_id.append(index[0][0][0][ii])
            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    #  plot_road_map(road_map, sample_x, sample_y)

    return road_map


def dijkstra_planning(sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(sx, sy, 0.0, -1)
    ngoal = Node(gx, gy, 0.0, -1)

    openset, closedset = dict(), dict()
    openset[len(road_map) - 2] = nstart

    while True:
        if len(openset) == 0:
            print("Cannot find path")
            break

        print(len(openset), len(closedset))

        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]
        print("current", current, c_id)
        #  input()

        # show graph
        plt.plot(current.x, current.y, "xc")
        if len(closedset.keys()) % 10 == 0:
            plt.pause(0.001)
            matplotrecorder.save_frame()

        if c_id == (len(road_map) - 1):
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            print(i, n_id)
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.sqrt(dx**2 + dy**2)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

            #  if not verify_node(node, obmap, minx, miny, maxx, maxy):
            #  continue

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    # generate final course
    rx, ry = [ngoal.x], [ngoal.y]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x)
        ry.append(n.y)
        pind = n.pind

    return rx, ry


def plot_road_map(road_map, sample_x, sample_y):

    for i in range(len(road_map)):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


def sample_points(sx, sy, gx, gy, rr, ox, oy):
    maxx = max(ox)
    maxy = max(oy)
    minx = min(ox)
    miny = min(oy)

    sample_x, sample_y = [], []

    nns = pyfastnns.NNS(np.vstack((ox, oy)).T)

    while len(sample_x) <= N_SAMPLE:
        tx = (random.random() - minx) * (maxx - minx)
        ty = (random.random() - miny) * (maxy - miny)

        index, dist = nns.search(np.matrix([tx, ty]).T)

        if dist[0] >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    robot_size = 5.0  # [m]

    ox = []
    oy = []

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

    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "xr")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")

    rx, ry = PRM_planning(sx, sy, gx, gy, ox, oy, robot_size)

    plt.plot(rx, ry, "-r")

    for i in range(20):
        matplotrecorder.save_frame()
    plt.show()

    matplotrecorder.save_movie("animation.gif", 0.1)


if __name__ == '__main__':
    main()
