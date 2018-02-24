"""

Iterative Closet Point (ICP) SLAM example

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
import matplotlib.pyplot as plt


def update_homogenerous_matrix(Hin, R, T):

    H = np.matrix(np.zeros((3, 3)))  # translation vector

    H[0, 0] = R[0, 0]
    H[1, 0] = R[1, 0]
    H[0, 1] = R[0, 1]
    H[1, 1] = R[1, 1]
    H[2, 2] = 1.0

    H[0, 2] = T[0, 0]
    H[1, 2] = T[1, 0]

    if Hin is None:
        return H
    else:
        return Hin * H


def ICP_matching(pdata, data):
    H = None  # homogeneraous transformation matrix

    #  ICP
    EPS = 0.0001
    maxIter = 100

    dError = 1000.0
    preError = 1000.0
    count = 0

    while dError >= EPS:
        count += 1

        error = nearest_neighbor_assosiation(pdata, data)
        Rt, Tt = SVD_motion_estimation(pdata, data)

        data = (Rt * data) + Tt

        H = update_homogenerous_matrix(H, Rt, Tt)

        dError = abs(preError - error)
        preError = error

        if dError <= EPS:
            print("Converge", dError, count)
            break
        elif maxIter <= count:
            break

    R = np.matrix(H[0:2, 0:2])
    T = np.matrix(H[0:2, 2])

    return R, T


def nearest_neighbor_assosiation(pdata, data):

    ddata = pdata - data

    d = np.linalg.norm(ddata, axis=0)

    error = sum(d)

    for i in range(data.shape[1]):
        for ii in range(data.shape[1]):
            print(i)

    return error


def SVD_motion_estimation(pdata, data):

    pm = np.matrix(np.mean(pdata, axis=1))
    cm = np.matrix(np.mean(data, axis=1))

    pshift = np.matrix(pdata - pm)
    cshift = np.matrix(data - cm)

    W = cshift * pshift.T
    u, s, vh = np.linalg.svd(W)

    R = (u * vh).T
    t = pm - R * cm

    return R, t


def main():
    print(__file__ + " start!!")

    # simulation parameters
    nPoint = 10
    fieldLength = 50.0
    motion = [0.5, 1.0, math.radians(-40.0)]  # movement [x[m],y[m],yaw[deg]]
    #  transitionSigma=0.01;%並進方向の移動誤差標準偏差[m]
    #  thetaSigma=1;   %回転方向の誤差標準偏差[deg]

    # previous point data
    px = (np.random.rand(nPoint) - 0.5) * fieldLength
    py = (np.random.rand(nPoint) - 0.5) * fieldLength

    cx = [math.cos(motion[2]) * x - math.sin(motion[2]) * y + motion[0]
          for (x, y) in zip(px, py)]
    cy = [math.sin(motion[2]) * x + math.cos(motion[2]) * y + motion[1]
          for (x, y) in zip(px, py)]

    pdata = np.matrix(np.vstack((px, py)))
    #  print(pdata)
    data = np.matrix(np.vstack((cx, cy)))
    #  print(data)
    odata = data[:, :]

    R, T = ICP_matching(pdata, data)

    fdata = (R * odata) + T

    plt.plot(px, py, ".b")
    plt.plot(cx, cy, ".r")
    plt.plot(fdata[0, :], fdata[1, :], "xk")
    plt.plot(0.0, 0.0, "xr")
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
