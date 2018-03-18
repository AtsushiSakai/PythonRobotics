"""

Histogram Filter 2D localization example


In this simulation, x,y are unknown, yaw is known.

Initial position is not needed.

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
import matplotlib.pyplot as plt
import copy
from scipy.stats import norm
from scipy.ndimage import gaussian_filter

# Parameters
NOISE_RANGE = 2.0  # [m] 1σ range noise parameter
NOISE_SPEED = 0.5  # [m/s] 1σ speed noise parameter

EXTEND_AREA = 10.0  # [m] grid map extention length
SIM_TIME = 50.0  # simulation time [s]
DT = 0.1  # time tick [s]
MAX_RANGE = 10.0  # maximum observation range
MOTION_STD = 1.0
RANGE_STD = 3.0  # standard diviation for gaussian distribution

show_animation = True


class grid_map():

    def __init__(self):
        self.data = None
        self.xyreso = None
        self.minx = None
        self.miny = None
        self.maxx = None
        self.maxx = None


def histogram_filter_localization(gmap, u, z, yaw, dx, dy):

    gmap, dx, dy = motion_update(gmap, u, yaw, dx, dy)

    gmap = observation_update(gmap, z, RANGE_STD)

    return gmap.data, dx, dy


def observation_update(gmap, z, std):

    for iz in range(z.shape[0]):
        for ix in range(len(gmap.data)):
            for iy in range(len(gmap.data[ix])):

                # observation range
                zr = z[iz, 0]

                # predicted range
                x = ix * gmap.xyreso + gmap.minx
                y = iy * gmap.xyreso + gmap.miny
                d = math.sqrt((x - z[iz, 1])**2 + (y - z[iz, 2])**2)

                # likelihood
                pdf = (1.0 - norm.cdf(abs(d - zr), 0.0, std))
                gmap.data[ix][iy] *= pdf

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


def draw_heatmap(data, mx, my):
    maxp = max([max(igmap) for igmap in data])
    plt.pcolor(mx, my, data, vmax=maxp, cmap=plt.cm.Blues)
    plt.axis("equal")


def observation(xTrue, u, RFID):

    xTrue = motion_model(xTrue, u)

    z = np.matrix(np.zeros((0, 3)))

    for i in range(len(RFID[:, 0])):

        dx = xTrue[0, 0] - RFID[i, 0]
        dy = xTrue[1, 0] - RFID[i, 1]
        d = math.sqrt(dx**2 + dy**2)
        if d <= MAX_RANGE:
            # add noise to range observation
            dn = d + np.random.randn() * NOISE_RANGE
            zi = np.matrix([dn, RFID[i, 0], RFID[i, 1]])
            z = np.vstack((z, zi))

    # add noise to speed
    ud = u[:, :]
    ud[0] += np.random.randn() * NOISE_SPEED

    return xTrue, z, ud


def normalize_probability(gmap):

    sump = sum([sum(igmap) for igmap in gmap.data])

    for i in range(len(gmap.data)):
        for ii in range(len(gmap.data[i])):
            gmap.data[i][ii] /= sump

    return gmap


def init_gmap(xyreso):

    gmap = grid_map()

    gmap.xyreso = xyreso
    gmap.minx = -15.0
    gmap.miny = -5.0
    gmap.maxx = 15.0
    gmap.maxy = 25.0
    gmap.xw = int(round((gmap.maxx - gmap.minx) / gmap.xyreso))
    gmap.yw = int(round((gmap.maxy - gmap.miny) / gmap.xyreso))

    gmap.data = [[1.0 for i in range(gmap.yw)] for i in range(gmap.xw)]
    gmap = normalize_probability(gmap)

    return gmap


def map_shift(gmap, xshift, yshift):

    tgmap = copy.deepcopy(gmap.data)

    lenx = len(gmap.data)
    leny = len(gmap.data[0])

    for ix in range(lenx):
        for iy in range(leny):
            nix = ix + xshift
            niy = iy + yshift

            if nix >= 0 and nix < lenx and niy >= 0 and niy < leny:
                gmap.data[ix + xshift][iy + yshift] = tgmap[ix][iy]

    return gmap


def motion_update(gmap, u, yaw, dx, dy):

    dx += DT * math.cos(yaw) * u[0]
    dy += DT * math.sin(yaw) * u[0]

    xshift = dx // gmap.xyreso
    yshift = dy // gmap.xyreso

    if abs(xshift) >= 1.0 or abs(yshift) >= 1.0:
        gmap = map_shift(gmap, int(xshift), int(yshift))
        dx -= xshift * gmap.xyreso
        dy -= yshift * gmap.xyreso

    gmap.data = gaussian_filter(gmap.data, sigma=MOTION_STD)

    return gmap, dx, dy


def main():
    print(__file__ + " start!!")

    xyreso = 0.5  # xy grid resolution

    # RFID positions [x, y]
    RFID = np.array([[10.0, 0.0],
                     [10.0, 10.0],
                     [0.0, 15.0],
                     [-5.0, 20.0]])

    time = 0.0

    xTrue = np.matrix(np.zeros((4, 1)))

    gmap = init_gmap(xyreso)

    dx, dy = 0.0, 0.0

    mx, my = np.mgrid[slice(gmap.minx - gmap.xyreso / 2.0, gmap.maxx + gmap.xyreso / 2.0, gmap.xyreso),
                      slice(gmap.miny - gmap.xyreso / 2.0, gmap.maxy + gmap.xyreso / 2.0, gmap.xyreso)]

    while SIM_TIME >= time:
        time += DT

        u = calc_input()

        # Orientation is known in this simulation
        yaw = xTrue[2, 0]
        xTrue, z, ud = observation(xTrue, u, RFID)

        gmap.data, dx, dy = histogram_filter_localization(
            gmap, u, z, yaw, dx, dy)

        if show_animation:
            plt.cla()
            draw_heatmap(gmap.data, mx, my)
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
