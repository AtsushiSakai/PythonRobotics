"""

Histogram Filter 2D localization example

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

EXTEND_AREA = 10.0  # [m] grid map extention length
SIM_TIME = 50.0  # simulation time [s]
DT = 0.1  # time tick [s]
MAX_RANGE = 10.0  # maximum observation range

show_animation = True


def observation_update(gmap, z, std, xyreso, minx, miny):

    for iz in range(z.shape[0]):
        for ix in range(len(gmap)):
            for iy in range(len(gmap[ix])):

                zr = z[iz, 0]
                x = ix * xyreso + minx
                y = iy * xyreso + miny

                d = math.sqrt((x - z[iz, 1])**2 + (y - z[iz, 2])**2)

                pdf = (1.0 - norm.cdf(abs(d - zr), 0.0, std))
                gmap[ix][iy] *= pdf

    gmap = normalize_probability(gmap)

    return gmap


def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.matrix([v, yawrate]).T
    return u


def motion_model(x, u):

    F = np.matrix([[1.0, 0, 0, 0],
                   [0, 1.0, 0, 0],
                   [0, 0, 1.0, 0],
                   [0, 0, 0, 0]])

    B = np.matrix([[DT * math.cos(x[2, 0]), 0],
                   [DT * math.sin(x[2, 0]), 0],
                   [0.0, DT],
                   [1.0, 0.0]])

    x = F * x + B * u

    return x


def draw_heatmap(data, minx, maxx, miny, maxy, xyreso):
    x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                    slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
    plt.pcolor(x, y, data, vmax=1.0, cmap=plt.cm.Blues)
    plt.axis("equal")


def observation(xTrue, u, RFID):

    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.matrix(np.zeros((0, 3)))

    for i in range(len(RFID[:, 0])):

        dx = xTrue[0, 0] - RFID[i, 0]
        dy = xTrue[1, 0] - RFID[i, 1]
        d = math.sqrt(dx**2 + dy**2)
        if d <= MAX_RANGE:
            dn = d
            zi = np.matrix([dn, RFID[i, 0], RFID[i, 1]])
            z = np.vstack((z, zi))

    return xTrue, z


def normalize_probability(gmap):

    sump = sum([sum(igmap) for igmap in gmap])
    #  print(sump)

    for i in range(len(gmap)):
        for ii in range(len(gmap[i])):
            gmap[i][ii] /= sump

    return gmap


def init_gmap(xyreso):

    minx = -15.0
    miny = -5.0
    maxx = 15.0
    maxy = 25.0
    xw = int(round((maxx - minx) / xyreso))
    yw = int(round((maxy - miny) / xyreso))

    gmap = [[1.0 for i in range(yw)] for i in range(xw)]
    gmap = normalize_probability(gmap)

    return gmap, minx, maxx, miny, maxy,


def main():
    print(__file__ + " start!!")

    xyreso = 0.5  # xy grid resolution
    STD = 1.0  # standard diviation for gaussian distribution

    # RFID positions [x, y]
    RFID = np.array([[10.0, 0.0],
                     [10.0, 10.0],
                     [0.0, 15.0],
                     [-5.0, 20.0]])

    time = 0.0

    xTrue = np.matrix(np.zeros((4, 1)))

    gmap, minx, maxx, miny, maxy = init_gmap(xyreso)

    while SIM_TIME >= time:
        time += DT

        u = calc_input()
        xTrue, z = observation(xTrue, u, RFID)

        gmap = observation_update(gmap, z, STD, xyreso, minx, miny)

        if show_animation:
            plt.cla()
            draw_heatmap(gmap, minx, maxx, miny, maxy, xyreso)
            plt.plot(xTrue[0, :], xTrue[1, :], "xr")
            plt.plot(RFID[:, 0], RFID[:, 1], ".k")
            for i in range(z.shape[0]):
                plt.plot([xTrue[0, :], z[i, 1]], [
                         xTrue[1, :], z[i, 2]], "-k")
            plt.title("Time[s]:" + str(time)[0: 4])
            plt.pause(0.1)

    print("Done")


if __name__ == '__main__':
    main()
