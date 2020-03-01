"""

Path Planning with B-Spline

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as scipy_interpolate


def b_spline_planning(x, y, sn, degree=3):
    t = range(len(x))
    x_tup = scipy_interpolate.splrep(t, x, k=degree)
    y_tup = scipy_interpolate.splrep(t, y, k=degree)

    x_list = list(x_tup)
    xl = x.tolist()
    x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

    y_list = list(y_tup)
    yl = y.tolist()
    y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, sn)
    rx = scipy_interpolate.splev(ipl_t, x_list)
    ry = scipy_interpolate.splev(ipl_t, y_list)

    return rx, ry


def main():
    print(__file__ + " start!!")
    # way points
    way_x = np.array([-1.0, 3.0, 4.0, 2.0, 1.0])
    way_y = np.array([0.0, -3.0, 1.0, 1.0, 3.0])
    sn = 100  # sampling number

    rx, ry = b_spline_planning(way_x, way_y, sn)

    # show results
    plt.plot(way_x, way_y, '-og', label="way points")
    plt.plot(rx, ry, '-r', label="B-Spline path")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
