"""
Grid based sweep planner

author: Atsushi Sakai
"""

import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.append(os.path.relpath("../../Mapping/grid_map_lib/"))
from grid_map_lib import GridMap


class SweepSearcher:

    def __init__(self, mdirection, sdirection, xinds_miny, miny):
        self.moving_direction = mdirection  # +1 or -1
        self.sweep_direction = sdirection  # +1 or -1
        self.turing_window = []
        self.update_turning_window()
        self.xinds_miny = xinds_miny
        self.miny = miny

    def move_target_grid(self, cxind, cyind, gmap):

        nxind = self.moving_direction + cxind
        nyind = cyind

        # found safe grid
        if not gmap.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5):
            return nxind, nyind
        else:  # occupided
            ncxind, ncyind = self.find_safe_turning_grid(cxind, cyind, gmap)
            if not ncxind and not ncyind:
                # moving backward
                ncxind = -self.moving_direction + cxind
                ncyind = cyind
                if gmap.check_occupied_from_xy_index(ncxind, ncyind):
                    # moved backward, but the grid is occupied by obstacle
                    return None, None
            else:
                self.swap_moving_direction()
            return ncxind, ncyind

    def find_safe_turning_grid(self, cxind, cyind, gmap):

        for (dxind, dyind) in self.turing_window:

            nxind = dxind + cxind
            nyind = dyind + cyind

            # found safe grid
            if not gmap.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5):
                print(dxind, dyind)
                return nxind, nyind

        return None, None

    def is_search_done(self, gmap):
        for ix in self.xinds_miny:
            if not gmap.check_occupied_from_xy_index(ix, self.miny, occupied_val=0.5):
                return False

        # all lower grid is occupied
        return True

    def update_turning_window(self):
        self.turing_window = [
            (self.moving_direction, 0.0),
            (self.moving_direction, self.sweep_direction),
            (0, self.sweep_direction),
            (-self.moving_direction, self.sweep_direction),
        ]

    def swap_moving_direction(self):
        self.moving_direction *= -1
        self.update_turning_window()


def find_sweep_direction_and_start_posi(ox, oy, start_posi):
    # find sweep_direction
    maxd_id = None
    maxd = 0.0
    vec = [0.0, 0.0]
    for i in range(len(ox) - 1):
        dx = ox[i + 1] - ox[i]
        dy = oy[i + 1] - oy[i]
        d = np.sqrt(dx ** 2 + dy ** 2)

        if d > maxd:
            maxd_id = i
            maxd = d
            vec = [dx, dy]

    # find sweep start posi
    d1 = np.sqrt((ox[maxd_id] - start_posi[0]) ** 2 +
                 (oy[maxd_id] - start_posi[1]) ** 2)
    d2 = np.sqrt((ox[maxd_id + 1] - start_posi[0]) ** 2 +
                 (oy[maxd_id + 1] - start_posi[1]) ** 2)

    if d2 >= d1:  # first point is near
        sweep_start_posi = [ox[maxd_id], oy[maxd_id]]
    else:
        sweep_start_posi = [ox[maxd_id + 1], oy[maxd_id + 1]]
        vec = [-1.0 * iv for iv in vec]  # reverse direction

    return vec, sweep_start_posi


def convert_grid_coordinate(ox, oy, sweep_vec, sweep_start_posi):
    tx = [ix - sweep_start_posi[0] for ix in ox]
    ty = [iy - sweep_start_posi[1] for iy in oy]

    th = math.atan2(sweep_vec[1], sweep_vec[0])

    c = np.cos(-th)
    s = np.sin(-th)

    rx = [ix * c - iy * s for (ix, iy) in zip(tx, ty)]
    ry = [ix * s + iy * c for (ix, iy) in zip(tx, ty)]

    return rx, ry


def convert_global_coordinate(x, y, sweep_vec, sweep_start_posi):
    th = math.atan2(sweep_vec[1], sweep_vec[0])
    c = np.cos(th)
    s = np.sin(th)

    tx = [ix * c - iy * s for (ix, iy) in zip(x, y)]
    ty = [ix * s + iy * c for (ix, iy) in zip(x, y)]

    rx = [ix + sweep_start_posi[0] for ix in tx]
    ry = [iy + sweep_start_posi[1] for iy in ty]

    return rx, ry


def search_free_lower_y_grid_index(grid_map):
    miny = None
    xinds = []

    for iy in range(grid_map.height):
        for ix in range(grid_map.width):
            if not grid_map.check_occupied_from_xy_index(ix, iy):
                miny = iy
                xinds.append(ix)
        if miny:
            break

    return xinds, miny


def setup_grid_map(ox, oy, reso):
    offset_grid = 10

    width = math.ceil((max(ox) - min(ox)) / reso) + offset_grid
    height = math.ceil((max(oy) - min(oy)) / reso) + offset_grid
    center_x = np.mean(ox)
    center_y = np.mean(oy)

    grid_map = GridMap(width, height, reso, center_x, center_y)

    grid_map.set_value_from_polygon(ox, oy, 1.0, inside=False)

    # fill grid
    # for i in range(len(ox) - 1):
    #     grid_map.set_value_from_xy_pos(ox[i], oy[i], 1.0)
    #
    #     x, y = ox[i], oy[i]
    #     th = math.atan2(oy[i + 1] - oy[i], ox[i + 1] - ox[i])
    #     d = np.sqrt((x - ox[i + 1])**2 + (y - oy[i + 1])**2)
    #
    #     while d > reso:
    #         x += np.cos(th) * reso
    #         y += np.sin(th) * reso
    #         d = np.sqrt((x - ox[i + 1])**2 + (y - oy[i + 1])**2)
    #
    #     grid_map.set_value_from_xy_pos(x, y, 1.0)

    xinds, miny = search_free_lower_y_grid_index(grid_map)

    # grid_map.plot_gridmap()

    return grid_map, xinds, miny


def sweep_path_search(sweep_searcher, gmap, start_posi):
    sx, sy = start_posi[0], start_posi[1]
    # print(sx, sy)

    # search start grid
    cxind, cyind = gmap.get_xy_index_from_xy_pos(sx, sy)
    if gmap.check_occupied_from_xy_index(cxind, cyind):
        cxind, cyind = sweep_searcher.find_safe_turning_grid(cxind, cyind, gmap)
    gmap.set_value_from_xy_index(cxind, cyind, 0.5)

    px, py = [], []

    # fig, ax = plt.subplots()

    while True:

        cxind, cyind = sweep_searcher.move_target_grid(cxind, cyind, gmap)

        if sweep_searcher.is_search_done(gmap) or (not cxind and not cyind):
            print("Done")
            break

        x, y = gmap.calc_grid_central_xy_position_from_xy_index(
            cxind, cyind)

        px.append(x)
        py.append(y)

        gmap.set_value_from_xy_index(cxind, cyind, 0.5)

        # gmap.plot_grid_map(ax=ax)
        # plt.pause(0.1)

    gmap.plot_grid_map()

    return px, py


def planning(ox, oy, reso, start_posi):
    sweep_vec, sweep_start_posi = find_sweep_direction_and_start_posi(
        ox, oy, start_posi)

    rox, roy = convert_grid_coordinate(ox, oy, sweep_vec, sweep_start_posi)

    moving_direction = 1
    sweeping_direction = -1

    gmap, xinds_miny, miny = setup_grid_map(rox, roy, reso)

    sweep_searcher = SweepSearcher(moving_direction, sweeping_direction, xinds_miny, miny)

    px, py = sweep_path_search(sweep_searcher, gmap, start_posi)

    rx, ry = convert_global_coordinate(px, py, sweep_vec, sweep_start_posi)

    return rx, ry


def main():
    print("start!!")

    start_posi = [0.0, 0.0]

    ox = [0.0, 20.0, 50.0, 100.0, 130.0, 40.0, 0.0]
    oy = [0.0, -20.0, 0.0, 30.0, 60.0, 80.0, 0.0]
    reso = 5.0

    px, py = planning(ox, oy, reso, start_posi)

    plt.subplots()

    plt.plot(start_posi[0], start_posi[1], "or")
    plt.plot(ox, oy, "-xb")
    plt.plot(px, py, "-r")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

    print("done!!")


if __name__ == '__main__':
    main()
