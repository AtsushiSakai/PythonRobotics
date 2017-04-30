#!/usr/bin/python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import random

delta = 0.1
minXY = -5.0
maxXY = 5.0
nContour = 50
alpha = 0.01


def Jacob(state):
    u"""
    jacobi matrix of Himmelblau's function
    """
    x = state[0, 0]
    y = state[0, 1]
    dx = 4 * x ** 3 + 4 * x * y - 44 * x + 2 * x + 2 * y ** 2 - 14
    dy = 2 * x ** 2 + 4 * x * y + 4 * y ** 3 - 26 * y - 22
    J = np.matrix([dx, dy])
    return J


def HimmelblauFunction(x, y):
    u"""
    Himmelblau's function
    see Himmelblau's function - Wikipedia, the free encyclopedia
    http://en.wikipedia.org/wiki/Himmelblau%27s_function
    """
    return (x ** 2 + y - 11) ** 2 + (x + y ** 2 - 7) ** 2


def ConstrainFunction(x):
    return (2.0 * x + 1.0)


def CreateMeshData():
    x = np.arange(minXY, maxXY, delta)
    y = np.arange(minXY, maxXY, delta)
    X, Y = np.meshgrid(x, y)
    Z = [HimmelblauFunction(ix, iy) for (ix, iy) in zip(X, Y)]
    return(X, Y, Z)


def SteepestDescentMethod(start, Jacob):
    u"""
    Steepest Descent Method Optimization
    """

    result = start
    x = start

    while 1:
        J = Jacob(x)
        sumJ = np.sum(abs(alpha * J))
        if sumJ <= 0.01:
            print("OK")
            break

        x = x - alpha * J
        result = np.vstack((result, x))

    return result


# Main
start = np.matrix([random.uniform(minXY, maxXY), random.uniform(minXY, maxXY)])

result = SteepestDescentMethod(start, Jacob)
(X, Y, Z) = CreateMeshData()
CS = plt.contour(X, Y, Z, nContour)

Xc = np.arange(minXY, maxXY, delta)
Yc = [ConstrainFunction(x) for x in Xc]

plt.plot(start[0, 0], start[0, 1], "xr")

plt.plot(result[:, 0], result[:, 1], "-r")

plt.axis([minXY, maxXY, minXY, maxXY])
plt.show()
