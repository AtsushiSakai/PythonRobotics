"""

2D grid map sample

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import matplotlib.pyplot as plt

AREA_WIDTH = 30.0


def generate_ray_casting_grid_map(ox, oy, reso):

    minx = min(ox) - AREA_WIDTH / 2.0
    miny = min(oy) - AREA_WIDTH / 2.0
    maxx = max(ox) + AREA_WIDTH / 2.0
    maxy = max(oy) + AREA_WIDTH / 2.0
    xw = round((maxx - minx) / reso)
    yw = round((maxy - miny) / reso)

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            pmap[ix][iy] = x + y

    draw_heatmap(pmap)
    plt.show()


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)


def main():
    print(__file__ + " start!!")

    ox = [0.0]
    oy = [0.0]
    reso = 1.0

    generate_ray_casting_grid_map(ox, oy, reso)


if __name__ == '__main__':
    main()
