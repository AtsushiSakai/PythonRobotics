"""

Lookup Table generation for model predictive trajectory generator

author: Atsushi Sakai

"""
from matplotlib import pyplot as plt
import numpy as np
import math
import model_predictive_trajectory_generator as planner
import motion_model
import pandas as pd


def calc_states_list():
    maxyaw = np.deg2rad(-30.0)

    x = np.arange(10.0, 30.0, 5.0)
    y = np.arange(0.0, 20.0, 2.0)
    yaw = np.arange(-maxyaw, maxyaw, maxyaw)

    states = []
    for iyaw in yaw:
        for iy in y:
            for ix in x:
                states.append([ix, iy, iyaw])
    print("nstate:", len(states))

    return states


def search_nearest_one_from_lookuptable(tx, ty, tyaw, lookuptable):
    mind = float("inf")
    minid = -1

    for (i, table) in enumerate(lookuptable):

        dx = tx - table[0]
        dy = ty - table[1]
        dyaw = tyaw - table[2]
        d = math.sqrt(dx ** 2 + dy ** 2 + dyaw ** 2)
        if d <= mind:
            minid = i
            mind = d

    # print(minid)

    return lookuptable[minid]


def save_lookup_table(fname, table):
    mt = np.array(table)
    print(mt)
    # save csv
    df = pd.DataFrame()
    df["x"] = mt[:, 0]
    df["y"] = mt[:, 1]
    df["yaw"] = mt[:, 2]
    df["s"] = mt[:, 3]
    df["km"] = mt[:, 4]
    df["kf"] = mt[:, 5]
    df.to_csv(fname, index=None)

    print("lookup table file is saved as " + fname)


def generate_lookup_table():
    states = calc_states_list()
    k0 = 0.0

    # x, y, yaw, s, km, kf
    lookuptable = [[1.0, 0.0, 0.0, 1.0, 0.0, 0.0]]

    for state in states:
        bestp = search_nearest_one_from_lookuptable(
            state[0], state[1], state[2], lookuptable)

        target = motion_model.State(x=state[0], y=state[1], yaw=state[2])
        init_p = np.array(
            [math.sqrt(state[0] ** 2 + state[1] ** 2), bestp[4], bestp[5]]).reshape(3, 1)

        x, y, yaw, p = planner.optimize_trajectory(target, k0, init_p)

        if x is not None:
            print("find good path")
            lookuptable.append(
                [x[-1], y[-1], yaw[-1], float(p[0]), float(p[1]), float(p[2])])

    print("finish lookup table generation")

    save_lookup_table("lookuptable.csv", lookuptable)

    for table in lookuptable:
        xc, yc, yawc = motion_model.generate_trajectory(
            table[3], table[4], table[5], k0)
        plt.plot(xc, yc, "-r")
        xc, yc, yawc = motion_model.generate_trajectory(
            table[3], -table[4], -table[5], k0)
        plt.plot(xc, yc, "-r")

    plt.grid(True)
    plt.axis("equal")
    plt.show()

    print("Done")


def main():
    generate_lookup_table()


if __name__ == '__main__':
    main()
