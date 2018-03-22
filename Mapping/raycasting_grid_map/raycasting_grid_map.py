"""

Ray casting 2D grid map example

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
import matplotlib.pyplot as plt

EXTEND_AREA = 10.0

show_animation = True


def calc_grid_map_config(ox, oy, xyreso):
    minx = round(min(ox) - EXTEND_AREA / 2.0)
    miny = round(min(oy) - EXTEND_AREA / 2.0)
    maxx = round(max(ox) + EXTEND_AREA / 2.0)
    maxy = round(max(oy) + EXTEND_AREA / 2.0)
    xw = int(round((maxx - minx) / xyreso))
    yw = int(round((maxy - miny) / xyreso))

    return minx, miny, maxx, maxy, xw, yw


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


def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0

    return angle


def precasting(minx, miny, xw, yw, xyreso, yawreso):

    precast = [[] for i in range(int(round((math.pi * 2.0) / yawreso)) + 1)]

    for ix in range(xw):
        for iy in range(yw):
            px = ix * xyreso + minx
            py = iy * xyreso + miny

            d = math.sqrt(px**2 + py**2)
            angle = atan_zero_to_twopi(py, px)
            angleid = int(math.floor(angle / yawreso))

            pc = precastDB()

            pc.px = px
            pc.py = py
            pc.d = d
            pc.ix = ix
            pc.iy = iy
            pc.angle = angle

            precast[angleid].append(pc)

    return precast


def generate_ray_casting_grid_map(ox, oy, xyreso, yawreso):

    minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(ox, oy, xyreso)

    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    precast = precasting(minx, miny, xw, yw, xyreso, yawreso)

    for (x, y) in zip(ox, oy):

        d = math.sqrt(x**2 + y**2)
        angle = atan_zero_to_twopi(y, x)
        angleid = int(math.floor(angle / yawreso))

        gridlist = precast[angleid]

        ix = int(round((x - minx) / xyreso))
        iy = int(round((y - miny) / xyreso))

        for grid in gridlist:
            if grid.d > d:
                pmap[grid.ix][grid.iy] = 0.5

        pmap[ix][iy] = 1.0

    return pmap, minx, maxx, miny, maxy, xyreso


def draw_heatmap(data, minx, maxx, miny, maxy, xyreso):
    x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                    slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
    plt.pcolor(x, y, data, vmax=1.0, cmap=plt.cm.Blues)
    plt.axis("equal")


def main():
    print(__file__ + " start!!")

    xyreso = 0.25  # x-y grid resolution [m]
    yawreso = math.radians(10.0)  # yaw angle resolution [rad]

    for i in range(5):
        ox = (np.random.rand(4) - 0.5) * 10.0
        oy = (np.random.rand(4) - 0.5) * 10.0
        pmap, minx, maxx, miny, maxy, xyreso = generate_ray_casting_grid_map(
            ox, oy, xyreso, yawreso)
        if show_animation:
            plt.cla()
            draw_heatmap(pmap, minx, maxx, miny, maxy, xyreso)
            plt.plot(ox, oy, "xr")
            plt.plot(0.0, 0.0, "ob")
            plt.pause(1.0)


if __name__ == '__main__':
    main()
