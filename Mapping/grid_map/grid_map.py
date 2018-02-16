"""

2D grid map sample

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

AREA_WIDTH = 30.0


def generate_gaussian_grid_map(ox, oy, reso):

    minx = min(ox) - AREA_WIDTH / 2.0
    miny = min(oy) - AREA_WIDTH / 2.0
    maxx = max(ox) + AREA_WIDTH / 2.0
    maxy = max(oy) + AREA_WIDTH / 2.0
    xw = round((maxx - minx) / reso)
    yw = round((maxy - miny) / reso)

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    STD = 10.0  # standard diviation

    for ix in range(xw):
        for iy in range(yw):

            x = ix / reso + minx
            y = iy / reso + miny

            # Search minimum distance
            mindis = float("inf")
            for (iox, ioy) in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if mindis >= d:
                    mindis = d

            pdf = (1.0 - norm.cdf(mindis, 0.0, STD))
            pmap[ix][iy] = pdf

    draw_heatmap(pmap)
    plt.show()


class precastDB:

    def __init__(self):
        self.px = 0.0
        self.py = 0.0
        self.d = 0.0
        self.angle = 0.0
        self.ix = 0
        self.iy = 0

    def __str__(self):
        return str(self.px) + "," + str(self.py) + "," + str(self.d) + "," + str(self.angle)


def precasting(minx, miny, xw, yw, reso, yawreso):

    precast = [[] for i in range(round((math.pi * 2.0) / yawreso) + 1)]

    for ix in range(xw):
        for iy in range(yw):
            px = ix / reso + minx
            py = iy / reso + miny

            d = math.sqrt(px**2 + py**2)
            angle = math.atan2(py, px)
            if angle < 0.0:
                angle += math.pi * 2.0

            angleid = round(angle / yawreso)

            pc = precastDB()

            pc.px = px
            pc.py = py
            pc.d = d
            pc.ix = ix
            pc.iy = iy
            pc.angle = angle

            precast[angleid].append(pc)

    return precast


def generate_ray_casting_grid_map(ox, oy, reso):

    minx = min(ox) - AREA_WIDTH / 2.0
    miny = min(oy) - AREA_WIDTH / 2.0
    maxx = max(ox) + AREA_WIDTH / 2.0
    maxy = max(oy) + AREA_WIDTH / 2.0
    xw = round((maxx - minx) / reso)
    yw = round((maxy - miny) / reso)

    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    yawreso = math.radians(10.0)

    precast = precasting(minx, miny, xw, yw, reso, yawreso)

    for (x, y) in zip(ox, oy):

        d = math.sqrt(x**2 + y**2)
        angle = math.atan2(y, x)

        angleid = round(angle / yawreso)

        gridlist = precast[angleid]

        ix = round(x * reso - minx)
        iy = round(y * reso - miny)

        for grid in gridlist:

            if ix == grid.ix and iy == grid.iy:
                pmap[grid.ix][grid.iy] = 1.0
            elif grid.d > d:
                pmap[grid.ix][grid.iy] = 0.5

    draw_heatmap(pmap)
    plt.show()


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=1.0, cmap=plt.cm.Blues)
    plt.axis("equal")


def main():
    print(__file__ + " start!!")

    # obstacle positions
    ox = [-5.0, 5.0, 0.0, 10.0]
    oy = [0.0, 5.0, 10.0, -5.0]
    reso = 1.0

    #  generate_gaussian_grid_map(ox, oy, reso)
    generate_ray_casting_grid_map(ox, oy, reso)


if __name__ == '__main__':
    main()
