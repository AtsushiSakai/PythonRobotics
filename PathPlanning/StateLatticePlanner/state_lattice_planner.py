"""
State lattice planner

author: Atsushi Sakai
"""
from matplotlib import pyplot as plt
import numpy as np
import math
import model_predictive_trajectory_generator as planner
import motion_model
import pandas as pd


def calc_states_list():
    maxyaw = math.radians(-30.0)

    x = np.arange(1.0, 30.0, 5.0)
    y = np.arange(0.0, 20.0, 2.0)
    yaw = np.arange(-maxyaw, maxyaw, maxyaw)

    states = []
    for iyaw in yaw:
        for iy in y:
            for ix in x:
                states.append([ix, iy, iyaw])
    # print(len(states))

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


    return lookuptable[minid]


def get_lookup_table():

    data = pd.read_csv("lookuptable.csv")

    return np.array(data)


def generate_path(states, k0):

    # x, y, yaw, s, km, kf
    lookup_table = get_lookup_table()
    result = []

    for state in states:
        bestp = search_nearest_one_from_lookuptable(
            state[0], state[1], state[2], lookup_table)

        target = motion_model.State(x=state[0], y=state[1], yaw=state[2])
        init_p = np.matrix(
            [math.sqrt(state[0] ** 2 + state[1] ** 2), bestp[4], bestp[5]]).T

        x, y, yaw, p = planner.optimize_trajectory(target, k0, init_p)

        if x is not None:
            print("find good path")
            result.append(
                [x[-1], y[-1], yaw[-1], float(p[0]), float(p[1]), float(p[2])])

    print("finish path generation")
    return result



def calc_uniform_states():
    np = 5
    nh = 3
    d = 20
    a_min = - math.radians(45.0)
    a_max = math.radians(45.0)
    p_min = - math.radians(45.0)
    p_max = math.radians(45.0)
    x0 = 0.0
    y0 = 0.0
    yaw0 = 0.0

    states = []

    for i in range(np):
        for j in range(nh):
            n = i * nh + j
            a = a_min + (a_max - a_min) * i / (np - 1)
            xf = x0 + d * math.cos(a + yaw0)
            yf = y0 + d * math.sin(a + yaw0)
            yawf = yaw0+p_min+(p_max-p_min)*j/(nh-1)+a
            states.append([xf, yf, yawf])

    # print(states)
    # print(len(states))

    return states


def uniform_terminal_state_sampling():
    k0 = 0.0
    states = calc_uniform_states()
    result = generate_path(states, k0)

    for table in result:
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
    uniform_terminal_state_sampling()


if __name__ == '__main__':
    main()
