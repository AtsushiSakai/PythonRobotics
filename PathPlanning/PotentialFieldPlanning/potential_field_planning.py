"""

Potential Field based path planner


author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import matplotlib.pyplot as plt


def calc_potential_field(gx, gy, reso):
    minx = -10.0
    miny = -10.0
    maxx = 50.0
    maxy = 50.0
    xw = round(maxx - minx)
    yw = round(maxy - miny)

    pmap = [[0.0 for i in range(xw)] for i in range(yw)]

    for ix in range(xw):
        x = ix * reso + minx
        for iy in range(yw):
            y = iy * reso + miny

            ug = np.hypot(x - gx, y - gy)
            uo = 0.0
            uf = ug + uo

            pmap[ix][iy] = uf

    #  print(pmap)

    return pmap, minx, miny


def get_motion_model():
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion


def potential_field_planning(sx, sy, gx, gy, reso):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy, reso)

    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    rx, ry = [sx], [sy]
    motion = get_motion_model()
    while d >= reso:
        #  print(ix, iy)
        #  input()
        minp = float("inf")
        minix, miniy = -1, -1
        for i in range(len(motion)):
            inx = ix + motion[i][0]
            iny = iy + motion[i][1]
            p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny
        d = np.hypot(gx - xp, gy - yp)
        #  print(d, xp, yp)
        rx.append(xp)
        ry.append(yp)

    return rx, ry


def main():

    sx = 0.0
    sy = 10.0
    gx = 30.0
    gy = 30.0
    grid_size = 2.0  # [m]

    rx, ry = potential_field_planning(sx, sy, gx, gy, grid_size)

    plt.plot(rx, ry, "-r")

    plt.plot(sx, sy, "*r")
    plt.plot(gx, gy, "*r")
    plt.grid(True)
    plt.axis("equal")
    plt.show()

    print(__file__ + " start!!")


if __name__ == '__main__':
    main()
