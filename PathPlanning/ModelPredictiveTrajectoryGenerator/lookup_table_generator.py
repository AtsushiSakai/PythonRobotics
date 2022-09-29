"""

Lookup Table generation for model predictive trajectory generator

author: Atsushi Sakai

"""
import sys
import pathlib
path_planning_dir = pathlib.Path(__file__).parent.parent
sys.path.append(str(path_planning_dir))

from matplotlib import pyplot as plt
import numpy as np
import math

from ModelPredictiveTrajectoryGenerator import trajectory_generator,\
    motion_model


def calc_states_list(max_yaw=np.deg2rad(-30.0)):

    x = np.arange(10.0, 30.0, 5.0)
    y = np.arange(0.0, 20.0, 2.0)
    yaw = np.arange(-max_yaw, max_yaw, max_yaw)

    states = []
    for iyaw in yaw:
        for iy in y:
            for ix in x:
                states.append([ix, iy, iyaw])
    print("n_state:", len(states))

    return states


def search_nearest_one_from_lookup_table(tx, ty, tyaw, lookup_table):
    mind = float("inf")
    minid = -1

    for (i, table) in enumerate(lookup_table):

        dx = tx - table[0]
        dy = ty - table[1]
        dyaw = tyaw - table[2]
        d = math.sqrt(dx ** 2 + dy ** 2 + dyaw ** 2)
        if d <= mind:
            minid = i
            mind = d

    # print(minid)

    return lookup_table[minid]


def save_lookup_table(file_name, table):
    np.savetxt(file_name, np.array(table),
               fmt='%s', delimiter=",", header="x,y,yaw,s,km,kf", comments="")

    print("lookup table file is saved as " + file_name)


def generate_lookup_table():
    states = calc_states_list(max_yaw=np.deg2rad(-30.0))
    k0 = 0.0

    # x, y, yaw, s, km, kf
    lookup_table = [[1.0, 0.0, 0.0, 1.0, 0.0, 0.0]]

    for state in states:
        best_p = search_nearest_one_from_lookup_table(
            state[0], state[1], state[2], lookup_table)

        target = motion_model.State(x=state[0], y=state[1], yaw=state[2])
        init_p = np.array(
            [np.hypot(state[0], state[1]), best_p[4], best_p[5]]).reshape(3, 1)

        x, y, yaw, p = trajectory_generator.optimize_trajectory(target,
                                                                k0, init_p)

        if x is not None:
            print("find good path")
            lookup_table.append(
                [x[-1], y[-1], yaw[-1], float(p[0]), float(p[1]), float(p[2])])

    print("finish lookup table generation")

    save_lookup_table("lookup_table.csv", lookup_table)

    for table in lookup_table:
        x_c, y_c, yaw_c = motion_model.generate_trajectory(
            table[3], table[4], table[5], k0)
        plt.plot(x_c, y_c, "-r")
        x_c, y_c, yaw_c = motion_model.generate_trajectory(
            table[3], -table[4], -table[5], k0)
        plt.plot(x_c, y_c, "-r")

    plt.grid(True)
    plt.axis("equal")
    plt.show()

    print("Done")


def main():
    generate_lookup_table()


if __name__ == '__main__':
    main()
