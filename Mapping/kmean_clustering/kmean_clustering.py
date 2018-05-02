"""

Object clustering with k-mean algorithm

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import matplotlib.pyplot as plt
import random


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


def init_clusters(rx, ry, nc):

    clusters = Clusters(rx, ry, nc)

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


def kmean_clustering(rx, ry, nc):

    clusters = init_clusters(rx, ry, nc)
    clusters = calc_centroid(clusters)

    MAX_LOOP = 10
    DCOST_TH = 1.0
    pcost = 100.0
    for loop in range(MAX_LOOP):
        print("Loop:", loop)
        clusters, cost = update_clusters(clusters)
        clusters = calc_centroid(clusters)

        dcost = abs(cost - pcost)
        if dcost < DCOST_TH:
            break
        pcost = cost

    return clusters


def calc_raw_data():

    rx, ry = [], []

    cx = [0.0, 5.0]
    cy = [0.0, 5.0]
    npoints = 30
    rand_d = 3.0

    for (icx, icy) in zip(cx, cy):
        for _ in range(npoints):
            rx.append(icx + rand_d * (random.random() - 0.5))
            ry.append(icy + rand_d * (random.random() - 0.5))

    return rx, ry


def calc_labeled_points(ic, clusters):

    inds = np.array([i for i in range(clusters.ndata)
                     if clusters.labels[i] == ic])
    tx = np.array(clusters.x)
    ty = np.array(clusters.y)

    x = tx[inds]
    y = ty[inds]

    return x, y


def main():
    print(__file__ + " start!!")

    rx, ry = calc_raw_data()

    ncluster = 2
    clusters = kmean_clustering(rx, ry, ncluster)

    for ic in range(clusters.nlabel):
        x, y = calc_labeled_points(ic, clusters)
        plt.plot(x, y, "x")
    plt.plot(clusters.cx, clusters.cy, "o")
    plt.show()


if __name__ == '__main__':
    main()
