"""

author: Atsushi Sakai
"""
from sklearn.neural_network import MLPRegressor
from matplotlib import pyplot as plt
import numpy as np
import math
import motion_model


def calc_motion_param(maxs, ds, maxk, dk):
    s = np.arange(1.0, maxs, ds)
    km = np.arange(-maxk, maxk, dk)
    kf = np.arange(-maxk, maxk, dk)
    motion_param = []

    for p1 in s:
        for p2 in km:
            for p3 in kf:
                motion_param.append([p1, p2, p3])

    print("motion_param size is:", len(motion_param))
    return motion_param


def calc_state_param(motion_param):
    print("calc_motion_param")

    state_param = []

    for p in motion_param:
        x, y, yaw = motion_model.generate_last_state(p[0], p[1], p[2], 0.0)
        state_param.append([x, y, yaw])

    print("calc_state_param done", len(state_param))

    return state_param


def learn_motion_model():

    # calc motion param
    # s, k0, km, kf
    maxk = math.radians(45.0)
    dk = math.radians(1.0)
    maxs = 50.0
    ds = 5.0

    motion_param = calc_motion_param(maxs, ds, maxk, dk)
    state_param = calc_state_param(motion_param)
    reg = MLPRegressor(hidden_layer_sizes=(100, 100, 100))
    reg.fit(state_param, motion_param)
    print(reg.score(state_param, motion_param))

    x = 10.0
    y = 5.0
    yaw = math.radians(00.0)

    predict = reg.predict([[x, y, yaw]])
    print(predict)
    x, y, yaw = motion_model.generate_trajectory(
        predict[0, 0], predict[0, 1], predict[0, 2], 0.0)

    plt.plot(x, y, "-r", label="trajectory")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)
    plt.show()

    print("Done")


def main():
    learn_motion_model()


if __name__ == '__main__':
    main()
