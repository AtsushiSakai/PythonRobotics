"""

Histogram Filter 2D localization example


In this simulation, x,y are unknown, yaw is known.

Initial position is not needed.

author: Atsushi Sakai (@Atsushi_twi)

"""

import copy
import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter
from scipy.stats import norm

# Parameters
EXTEND_AREA = 10.0  # [m] grid map extended length
SIM_TIME = 50.0  # simulation time [s]
DT = 0.1  # time tick [s]
MAX_RANGE = 10.0  # maximum observation range
MOTION_STD = 1.0  # standard deviation for motion gaussian distribution
RANGE_STD = 3.0  # standard deviation for observation gaussian distribution

# grid map param
XY_RESO = 0.5  # xy grid resolution
MINX = -15.0
MINY = -5.0
MAXX = 15.0
MAXY = 25.0

# simulation parameters
NOISE_RANGE = 2.0  # [m] 1σ range noise parameter
NOISE_SPEED = 0.5  # [m/s] 1σ speed noise parameter

show_animation = True


class GridMap():

    def __init__(self):
        self.data = None
        self.xy_reso = None
        self.minx = None
        self.miny = None
        self.maxx = None
        self.maxx = None
        self.xw = None
        self.yw = None
        self.dx = 0.0  # movement distance
        self.dy = 0.0  # movement distance


def histogram_filter_localization(grid_map, u, z, yaw):
    grid_map = motion_update(grid_map, u, yaw)

    grid_map = observation_update(grid_map, z, RANGE_STD)

    return grid_map


def calc_gaussian_observation_pdf(gmap, z, iz, ix, iy, std):
    # predicted range
    x = ix * gmap.xy_reso + gmap.minx
    y = iy * gmap.xy_reso + gmap.miny
    d = math.hypot(x - z[iz, 1], y - z[iz, 2])

    # likelihood
    pdf = (1.0 - norm.cdf(abs(d - z[iz, 0]), 0.0, std))

    return pdf


def observation_update(gmap, z, std):
    for iz in range(z.shape[0]):
        for ix in range(gmap.xw):
            for iy in range(gmap.yw):
                gmap.data[ix][iy] *= calc_gaussian_observation_pdf(
                    gmap, z, iz, ix, iy, std)

    gmap = normalize_probability(gmap)

    return gmap


def calc_input():
    v = 1.0  # [m/s]
    yaw_rate = 0.1  # [rad/s]
    u = np.array([v, yaw_rate]).reshape(2, 1)
    return u


def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F @ x + B @ u

    return x


def draw_heat_map(data, mx, my):
    maxp = max([max(igmap) for igmap in data])
    plt.pcolor(mx, my, data, vmax=maxp, cmap=plt.cm.get_cmap("Blues"))
    plt.axis("equal")


def observation(xTrue, u, RFID):
    xTrue = motion_model(xTrue, u)

    z = np.zeros((0, 3))

    for i in range(len(RFID[:, 0])):

        dx = xTrue[0, 0] - RFID[i, 0]
        dy = xTrue[1, 0] - RFID[i, 1]
        d = math.hypot(dx, dy)
        if d <= MAX_RANGE:
            # add noise to range observation
            dn = d + np.random.randn() * NOISE_RANGE
            zi = np.array([dn, RFID[i, 0], RFID[i, 1]])
            z = np.vstack((z, zi))

    # add noise to speed
    ud = u[:, :]
    ud[0] += np.random.randn() * NOISE_SPEED

    return xTrue, z, ud


def normalize_probability(gmap):
    sump = sum([sum(igmap) for igmap in gmap.data])

    for ix in range(gmap.xw):
        for iy in range(gmap.yw):
            gmap.data[ix][iy] /= sump

    return gmap


def init_gmap(xy_reso, minx, miny, maxx, maxy):
    grid_map = GridMap()

    grid_map.xy_reso = xy_reso
    grid_map.minx = minx
    grid_map.miny = miny
    grid_map.maxx = maxx
    grid_map.maxy = maxy
    grid_map.xw = int(round((grid_map.maxx - grid_map.minx) / grid_map.xy_reso))
    grid_map.yw = int(round((grid_map.maxy - grid_map.miny) / grid_map.xy_reso))

    grid_map.data = [[1.0 for _ in range(grid_map.yw)] for _ in range(grid_map.xw)]
    grid_map = normalize_probability(grid_map)

    return grid_map


def map_shift(grid_map, x_shift, y_shift):
    tgmap = copy.deepcopy(grid_map.data)

    for ix in range(grid_map.xw):
        for iy in range(grid_map.yw):
            nix = ix + x_shift
            niy = iy + y_shift

            if 0 <= nix < grid_map.xw and 0 <= niy < grid_map.yw:
                grid_map.data[ix + x_shift][iy + y_shift] = tgmap[ix][iy]

    return grid_map


def motion_update(grid_map, u, yaw):
    grid_map.dx += DT * math.cos(yaw) * u[0]
    grid_map.dy += DT * math.sin(yaw) * u[0]

    x_shift = grid_map.dx // grid_map.xy_reso
    y_shift = grid_map.dy // grid_map.xy_reso

    if abs(x_shift) >= 1.0 or abs(y_shift) >= 1.0:  # map should be shifted
        grid_map = map_shift(grid_map, int(x_shift), int(y_shift))
        grid_map.dx -= x_shift * grid_map.xy_reso
        grid_map.dy -= y_shift * grid_map.xy_reso

    grid_map.data = gaussian_filter(grid_map.data, sigma=MOTION_STD)

    return grid_map


def calc_grid_index(gmap):
    mx, my = np.mgrid[slice(gmap.minx - gmap.xy_reso / 2.0, gmap.maxx + gmap.xy_reso / 2.0, gmap.xy_reso),
                      slice(gmap.miny - gmap.xy_reso / 2.0, gmap.maxy + gmap.xy_reso / 2.0, gmap.xy_reso)]

    return mx, my


def main():
    print(__file__ + " start!!")

    # RF_ID positions [x, y]
    RF_ID = np.array([[10.0, 0.0],
                      [10.0, 10.0],
                      [0.0, 15.0],
                      [-5.0, 20.0]])

    time = 0.0

    xTrue = np.zeros((4, 1))
    grid_map = init_gmap(XY_RESO, MINX, MINY, MAXX, MAXY)
    mx, my = calc_grid_index(grid_map)  # for grid map visualization

    while SIM_TIME >= time:
        time += DT
        print("Time:", time)

        u = calc_input()

        yaw = xTrue[2, 0]  # Orientation is known
        xTrue, z, ud = observation(xTrue, u, RF_ID)

        grid_map = histogram_filter_localization(grid_map, u, z, yaw)

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            draw_heat_map(grid_map.data, mx, my)
            plt.plot(xTrue[0, :], xTrue[1, :], "xr")
            plt.plot(RF_ID[:, 0], RF_ID[:, 1], ".k")
            for i in range(z.shape[0]):
                plt.plot([xTrue[0, :], z[i, 1]], [
                    xTrue[1, :], z[i, 2]], "-k")
            plt.title("Time[s]:" + str(time)[0: 4])
            plt.pause(0.1)

    print("Done")


if __name__ == '__main__':
    main()
