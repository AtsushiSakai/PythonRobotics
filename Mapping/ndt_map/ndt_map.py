
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

from Mapping.grid_map_lib.grid_map_lib import GridMap


class NDTMap:

    class NDT:

        def __init__(self):
            self.x = 0

    def __init__(self, ox, oy, resolution):
        """
        :param ox
        :param oy
        :param resolution: grid resolution [m]
        """
        width = int((max(ox) - min(ox))/resolution) + 3
        height = int((max(oy) - min(oy))/resolution) + 3
        center_x = np.mean(ox)
        center_y = np.mean(oy)
        self.ox = ox
        self.oy = oy

        self.grid_map = GridMap(width, height, resolution, center_x, center_y, self.NDT())

        self.grid_index_map = defaultdict(list)
        for i in range(len(ox)):
            grid_index = self.grid_map.calc_grid_index_from_xy_pos(ox[i], oy[i])
            self.grid_index_map[grid_index].append(i)



def main():
    print(__file__ + " start!!")

    ox, oy = create_dummy_observation_data()
    grid_resolution = 10.0
    ndt_map = NDTMap(ox, oy, grid_resolution)

    # plot raw observation
    plt.plot(ox, oy, ".r")

    # grid clustering
    [plt.plot(ox[inds], oy[inds], "x") for inds in ndt_map.grid_index_map.values()]

    plt.axis("equal")
    plt.show()


def create_dummy_observation_data():
    ox = []
    oy = []
    # left corridor
    for y in range(-50, 50):
        ox.append(-20.0)
        oy.append(y)
    # right corridor 1
    for y in range(-50, 0):
        ox.append(20.0)
        oy.append(y)
    # right corridor 2
    for x in range(20, 50):
        ox.append(x)
        oy.append(0)
    # right corridor 3
    for x in range(20, 50):
        ox.append(x)
        oy.append(x/2.0+10)
    # right corridor 4
    for y in range(20, 50):
        ox.append(20)
        oy.append(y)
    ox = np.array(ox)
    oy = np.array(oy)
    # Adding random noize
    ox += np.random.rand(len(ox)) * 1.0
    oy += np.random.rand(len(ox)) * 1.0
    return ox, oy


if __name__ == '__main__':
    main()
