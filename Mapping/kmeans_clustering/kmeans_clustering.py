"""

Object clustering with k-means algorithm

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import matplotlib.pyplot as plt
import random

show_animation = True


class Clusters:

    def __init__(self, x, y, nlabel):
        self.x = x
        self.y = y
        self.ndata = len(self.x)
        self.nlabel = nlabel
        self.labels = [random.randint(0, nlabel - 1)
                       for _ in range(self.ndata)]
        self.cx = [0.0 for _ in range(nlabel)]
        self.cy = [0.0 for _ in range(nlabel)]


def kmeans_clustering(rx, ry, nc):

    clusters = Clusters(rx, ry, nc)
    clusters = calc_centroid(clusters)

    MAX_LOOP = 10
    DCOST_TH = 0.1
    pcost = 100.0
    for loop in range(MAX_LOOP):
        #  print("Loop:", loop)
        clusters, cost = update_clusters(clusters)
        clusters = calc_centroid(clusters)

        dcost = abs(cost - pcost)
        if dcost < DCOST_TH:
            break
        pcost = cost

    return clusters


def calc_centroid(clusters):

    for ic in range(clusters.nlabel):
        x, y = calc_labeled_points(ic, clusters)
        ndata = len(x)
        clusters.cx[ic] = sum(x) / ndata
        clusters.cy[ic] = sum(y) / ndata

    return clusters


def update_clusters(clusters):
    cost = 0.0

    for ip in range(clusters.ndata):
        px = clusters.x[ip]
        py = clusters.y[ip]

        dx = [icx - px for icx in clusters.cx]
        dy = [icy - py for icy in clusters.cy]

        dlist = [math.sqrt(idx**2 + idy**2) for (idx, idy) in zip(dx, dy)]
        mind = min(dlist)
        min_id = dlist.index(mind)
        clusters.labels[ip] = min_id
        cost += min_id

    return clusters, cost


def calc_labeled_points(ic, clusters):

    inds = np.array([i for i in range(clusters.ndata)
                     if clusters.labels[i] == ic])
    tx = np.array(clusters.x)
    ty = np.array(clusters.y)

    x = tx[inds]
    y = ty[inds]

    return x, y


def calc_raw_data(cx, cy, npoints, rand_d):

    rx, ry = [], []

    for (icx, icy) in zip(cx, cy):
        for _ in range(npoints):
            rx.append(icx + rand_d * (random.random() - 0.5))
            ry.append(icy + rand_d * (random.random() - 0.5))

    return rx, ry


def update_positions(cx, cy):

    DX1 = 0.4
    DY1 = 0.5

    cx[0] += DX1
    cy[0] += DY1

    DX2 = -0.3
    DY2 = -0.5

    cx[1] += DX2
    cy[1] += DY2

    return cx, cy


def calc_association(cx, cy, clusters):

    inds = []

    for ic in range(len(cx)):
        tcx = cx[ic]
        tcy = cy[ic]

        dx = [icx - tcx for icx in clusters.cx]
        dy = [icy - tcy for icy in clusters.cy]

        dlist = [math.sqrt(idx**2 + idy**2) for (idx, idy) in zip(dx, dy)]
        min_id = dlist.index(min(dlist))
        inds.append(min_id)

    return inds


def main():
    print(__file__ + " start!!")

    cx = [0.0, 8.0]
    cy = [0.0, 8.0]
    npoints = 10
    rand_d = 3.0
    ncluster = 2
    sim_time = 15.0
    dt = 1.0
    time = 0.0

    while time <= sim_time:
        print("Time:", time)
        time += dt

        # simulate objects
        cx, cy = update_positions(cx, cy)
        rx, ry = calc_raw_data(cx, cy, npoints, rand_d)

        clusters = kmeans_clustering(rx, ry, ncluster)

        # for animation
        if show_animation:
            plt.cla()
            inds = calc_association(cx, cy, clusters)
            for ic in inds:
                x, y = calc_labeled_points(ic, clusters)
                plt.plot(x, y, "x")
            plt.plot(cx, cy, "o")
            plt.xlim(-2.0, 10.0)
            plt.ylim(-2.0, 10.0)
            plt.pause(dt)

    print("Done")


if __name__ == '__main__':
    main()
