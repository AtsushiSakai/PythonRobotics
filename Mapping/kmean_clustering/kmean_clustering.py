"""

Object clustering with k-mean algorithm


author: Atsushi Sakai (@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import random


class Cluster:

    def __init__(self):
        self.x = []
        self.y = []
        self.cx = None
        self.cy = None


def kmean_clustering(rx, ry, nc):

    minx, maxx = min(rx), max(rx)
    miny, maxy = min(ry), max(ry)

    clusters = [Cluster() for i in range(nc)]

    for c in clusters:
        c.cx = random.uniform(minx, maxx)
        c.cy = random.uniform(miny, maxy)

    return clusters


def calc_raw_data():

    rx, ry = [], []

    cx = [0.0, 5.0]
    cy = [0.0, 5.0]
    np = 30
    rand_d = 3.0

    for (icx, icy) in zip(cx, cy):
        for _ in range(np):
            rx.append(icx + rand_d * (random.random() - 0.5))
            ry.append(icy + rand_d * (random.random() - 0.5))

    return rx, ry


def main():
    print(__file__ + " start!!")

    rx, ry = calc_raw_data()

    ncluster = 2
    clusters = kmean_clustering(rx, ry, ncluster)

    for c in clusters:
        print(c.cx, c.cy)
        plt.plot(c.cx, c.cy, "x")

    plt.plot(rx, ry, ".")
    plt.show()


if __name__ == '__main__':
    main()
