"""

Object shape recognition with rectangle fitting

author: Atsushi Sakai (@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import math
import random
import numpy as np

show_animation = True


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

    T = np.linalg.inv(F).dot(G)

    cxe = float(T[0] / -2)
    cye = float(T[1] / -2)
    re = math.sqrt(cxe**2 + cye**2 - T[2])

    error = sum([np.hypot(cxe - ix, cye - iy) - re for (ix, iy) in zip(x, y)])

    return (cxe, cye, re, error)


def get_sample_points(cx, cy, cr, angle_reso):
    x, y, angle, r = [], [], [], []

    # points sampling
    for theta in np.arange(0.0, 2.0 * math.pi, angle_reso):
        nx = cx + cr * math.cos(theta)
        ny = cy + cr * math.sin(theta)
        nangle = math.atan2(ny, nx)
        nr = math.hypot(nx, ny) * random.uniform(0.95, 1.05)

        x.append(nx)
        y.append(ny)
        angle.append(nangle)
        r.append(nr)

    # ray casting filter
    rx, ry = ray_casting_filter(x, y, angle, r, angle_reso)

    return rx, ry


def ray_casting_filter(xl, yl, thetal, rangel, angle_reso):
    rx, ry = [], []
    rangedb = [float("inf") for _ in range(
        int(math.floor((math.pi * 2.0) / angle_reso)) + 1)]

    for i in range(len(thetal)):
        angleid = math.floor(thetal[i] / angle_reso)

        if rangedb[angleid] > rangel[i]:
            rangedb[angleid] = rangel[i]

    for i in range(len(rangedb)):
        t = i * angle_reso
        if rangedb[i] != float("inf"):
            rx.append(rangedb[i] * math.cos(t))
            ry.append(rangedb[i] * math.sin(t))

    return rx, ry


def plot_circle(x, y, size, color="-b"):
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(math.radians(d)) for d in deg]
    yl = [y + size * math.sin(math.radians(d)) for d in deg]
    plt.plot(xl, yl, color)


def main():

    # simulation parameters
    simtime = 15.0  # simulation time
    dt = 1.0  # time tick

    cx = -2.0  # initial x position of obstacle
    cy = -8.0  # initial y position of obstacle
    cr = 1.0  # obstacle radious
    theta = math.radians(30.0)  # obstacle moving direction
    angle_reso = math.radians(3.0)  # sensor angle resolution

    time = 0.0
    while time <= simtime:
        time += dt

        cx += math.cos(theta)
        cy += math.cos(theta)

        x, y = get_sample_points(cx, cy, cr, angle_reso)

        ex, ey, er, error = circle_fitting(x, y)
        print("Error:", error)

        if show_animation:
            plt.cla()
            plt.axis("equal")
            plt.plot(0.0, 0.0, "*r")
            plot_circle(cx, cy, cr)
            plt.plot(x, y, "xr")
            plot_circle(ex, ey, er, "-r")
            plt.pause(dt)

    print("Done")


if __name__ == '__main__':
    main()
