#!/usr/bin/python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import random
from math import *

delta = 0.1
minXY = -5.0
maxXY = 5.0
nContour = 50


def dfunc(d):
    x = d[0]
    y = d[1]
    l = d[2]
    dx = -2 * l + 4 * x * (x ** 2 + y - 11)
    dy = l + 2 * x * x + 2 * y - 22
    dl = -2 * x + y - 1
    return [dx, dy, dl]


def SampleFunc(x, y):
    return (x ** 2 + y - 11) ** 2


def ConstrainFunction(x):
    return (2.0 * x + 1.0)


def CreateMeshData():
    x = np.arange(minXY, maxXY, delta)
    y = np.arange(minXY, maxXY, delta)
    X, Y = np.meshgrid(x, y)
    Z = [SampleFunc(ix, iy) for (ix, iy) in zip(X, Y)]
    return(X, Y, Z)


# Main
start = np.matrix([random.uniform(minXY, maxXY),
                   random.uniform(minXY, maxXY), 0])

(X, Y, Z) = CreateMeshData()
CS = plt.contour(X, Y, Z, nContour)

Xc = np.arange(minXY, maxXY, delta)
Yc = [ConstrainFunction(x) for x in Xc]

#  plt.plot(start[0,0],start[0,1],"xr");
plt.plot(Xc, Yc, "-r")

#  X1 = fsolve(dfunc, [-3, -3, 10])
#  print(X1)
#  print(dfunc(X1))

# the answer from sympy
result = np.matrix([
    [-1, -1],
    [-1 + sqrt(11), -1 + 2 * sqrt(11)],
    [-sqrt(11) - 1, -2 * sqrt(11) - 1]])
print(result)

plt.plot(result[:, 0], result[:, 1], "or")

plt.axis([minXY, maxXY, minXY, maxXY])
plt.show()
