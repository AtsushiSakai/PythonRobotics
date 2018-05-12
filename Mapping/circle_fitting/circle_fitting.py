"""

Object shape recognition with circle fitting

author: Atsushi Sakai (@Atsushi_twi)

"""


import matplotlib.pyplot as plt
import math
import random
import numpy as np


def circle_fitting(x, y):
    """
    Circle Fitting with least squared
        input: point x-y positions
        output  cxe x center position
                cye y center position
                re  radius of circle
                error: prediction error
    """

    sumx = sum(x)
    sumy = sum(y)
    sumx2 = sum([ix ** 2 for ix in x])
    sumy2 = sum([iy ** 2 for iy in y])
    sumxy = sum([ix * iy for (ix, iy) in zip(x, y)])

    F = np.array([[sumx2, sumxy, sumx],
                  [sumxy, sumy2, sumy],
                  [sumx, sumy, len(x)]])

    G = np.array([[-sum([ix ** 3 + ix * iy ** 2 for (ix, iy) in zip(x, y)])],
                  [-sum([ix ** 2 * iy + iy ** 3 for (ix, iy) in zip(x, y)])],
                  [-sum([ix ** 2 + iy ** 2 for (ix, iy) in zip(x, y)])]])

    #  try:
    T = np.linalg.inv(F).dot(G)
    #  except:
    #  return (0, 0, float("inf"))

    cxe = float(T[0] / -2)
    cye = float(T[1] / -2)
    #  print (cxe,cye,T)
    #  try:
    re = math.sqrt(cxe**2 + cye**2 - T[2])
    #  except:
    #  return (cxe, cye, float("inf"))

    error = sum([np.hypot(cxe - ix, cye - iy) - re for (ix, iy) in zip(x, y)])
    #  print(error)

    return (cxe, cye, re, error)


def get_sample_points(cx, cy, r, angle_reso):
    x, y, angle, ran = [], [], [], []

    for theta in np.arange(0.0, 2.0 * math.pi, angle_reso):
        rn = r * random.uniform(1.0, 1.0)
        nx = cx + rn * math.cos(theta)
        ny = cy + rn * math.sin(theta)
        nangle = math.atan2(ny, nx)
        nr = math.hypot(nx, ny)

        occluded = False
        for i in range(len(angle)):
            if abs(angle[i] - nangle) <= angle_reso:
                if nr >= ran[i]:
                    occluded = True
                    break

        if not occluded:
            x.append(nx)
            y.append(ny)
            angle.append(nangle)
            ran.append(nr)

    return x, y


def plot_circle(x, y, size, color="-b"):
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(math.radians(d)) for d in deg]
    yl = [y + size * math.sin(math.radians(d)) for d in deg]
    plt.plot(xl, yl, color)


def main1():
    print(__file__ + " start!!")

    tcx = 1.0
    tcy = 2.0
    tr = 3.0
    np = 10

    x, y = get_sample_points(tcx, tcy, tr, np)

    cx, cy, r, error = circle_fitting(x, y)
    print("Error:", error)

    plot_circle(tcx, tcy, tr)
    plot_circle(cx, cy, r, color="-xr")
    plt.plot(x, y, "gx")

    plt.axis("equal")
    plt.show()


def main():

    time = 0.0
    simtime = 10.0
    dt = 1.0

    cx = -3.0
    cy = -5.0
    theta = math.radians(30.0)

    cr = 1.0
    angle_reso = math.radians(30.0)

    while time <= simtime:
        time += dt

        cx += math.cos(theta)
        cy += math.cos(theta)

        x, y = get_sample_points(cx, cy, cr, angle_reso)

        ex, ey, er, error = circle_fitting(x, y)
        print("Error:", error)

        plt.cla()
        plt.axis("equal")
        plt.plot(0.0, 0.0, "*r")
        plot_circle(cx, cy, cr)
        plt.plot(x, y, "xr")
        plot_circle(ex, ey, er, "-r")
        plt.pause(dt)


if __name__ == '__main__':
    #  main1()
    main()
