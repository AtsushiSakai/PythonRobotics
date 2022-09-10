"""

Path Planner with B-Spline

author: Atsushi Sakai (@Atsushi_twi)

"""

import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../utils/")


import numpy as np
import numpy
import matplotlib.pyplot as plt
import scipy.interpolate as interpolate

from utils.plot import plot_arrow, plot_curvature


def approximate_b_spline_path(x: list,
                              y: list,
                              n_path_points: int,
                              degree: int = 3
                              ) -> tuple:
    """
    approximate points with a B-Spline path

    Parameters
    ----------
    x : x position list of approximated points
    y : y position list of approximated points
    n_path_points : number of path points
    degree : (Optional) B Spline curve degree

    Returns
    -------
    x and y position list of the result path

    """
    dx, dy = np.diff(x), np.diff(y)
    distances = np.cumsum([np.hypot(idx, idy) for idx, idy in zip(dx, dy)])
    distances = np.concatenate(([0.0], distances))
    distances /= distances[-1]

    t = np.linspace(0.0, 1.0, len(x))
    t = np.r_[(t[0],)*(degree+2),
              t,
              (t[-1],)*(degree+2)]
    print(distances, x, t)
    spl_i_x = interpolate.make_lsq_spline(distances, x, t, k=degree)
    spl_i_y = interpolate.make_lsq_spline(distances, y, t, k=degree)

    sampled = np.linspace(0.0, distances[-1], n_path_points)
    x = spl_i_x(sampled)
    y = spl_i_y(sampled)
    dx = spl_i_x.derivative(1)(sampled)
    dy = spl_i_y.derivative(1)(sampled)
    heading = np.arctan2(dy, dx)
    ddx = spl_i_x.derivative(2)(sampled)
    ddy = spl_i_y.derivative(2)(sampled)
    curvature = (ddy * dx - ddx * dy) / np.power(dx * dx + dy * dy, 2.0/3.0)
    return x, y, heading, curvature

    # t = range(len(x))
    # x_tup = interpolate.splrep(t, x, k=degree)
    # y_tup = interpolate.splrep(t, y, k=degree)
    #
    # x_list = list(x_tup)
    # x_list[1] = x + [0.0, 0.0, 0.0, 0.0]
    #
    # y_list = list(y_tup)
    # y_list[1] = y + [0.0, 0.0, 0.0, 0.0]
    #
    # ipl_t = np.linspace(0.0, len(x) - 1, n_path_points)
    # rx = interpolate.splev(ipl_t, x_list)
    # ry = interpolate.splev(ipl_t, y_list)
    # dx_spline = interpolate.splder(x_tup, n=1)
    # dy_spline = interpolate.splder(y_tup, n=1)
    # dx = interpolate.splev(ipl_t, dx_spline)
    # dy = interpolate.splev(ipl_t, dy_spline)
    # heading = numpy.arctan2(dy, dx)
    # return rx, ry, heading


def interpolate_b_spline_path(x: list, y: list,
                              n_path_points: int,
                              degree: int = 3) -> tuple:
    """
    Interpolate x-y points with a B-Spline path

    Parameters
    ----------
    x : x positions of interpolated points
    y : y positions of interpolated points
    n_path_points : number of path points
    degree : B-Spline degree (default: 3)

    Returns
    -------
    x : x positions of the result path
    y : y positions of the result path
    heading : heading of the result path
    curvature : curvature of the result path

    """
    dx, dy = np.diff(x), np.diff(y)
    distances = np.cumsum([np.hypot(idx, idy) for idx, idy in zip(dx, dy)])
    distances = np.concatenate(([0.0], distances))

    spl_i_x = interpolate.make_interp_spline(distances, x, k=1)
    spl_i_y = interpolate.make_interp_spline(distances, y, k=degree)

    sampled = np.linspace(0.0, distances[-1], n_path_points)
    x = spl_i_x(sampled)
    y = spl_i_y(sampled)
    dx = spl_i_x.derivative(1)(sampled)
    dy = spl_i_y.derivative(1)(sampled)
    heading = np.arctan2(dy, dx)
    ddx = spl_i_x.derivative(2)(sampled)
    ddy = spl_i_y.derivative(2)(sampled)
    curvature = (ddy * dx - ddx * dy) / np.power(dx * dx + dy * dy, 2.0/3.0)
    return x, y, heading, curvature


def main():
    print(__file__ + " start!!")
    # way points
    # way_point_x = [-1.0, 3.0, 4.0, 2.0, 1.0]
    # way_point_y = [0.0, -3.0, 1.0, 1.0, 3.0]
    way_point_x = [-1.0, 3.0, 4.0, 2.0, 1.0]
    way_point_y = [0.0, -3.0, 1.0, 1.0, 3.0]
    n_course_point = 50  # sampling number

    rax, ray, heading = approximate_b_spline_path(way_point_x, way_point_y,
                                                  n_course_point)
    plt.plot(way_point_x, way_point_y, '-og', label="way points")
    plt.plot(rax, ray, '-r', label="Approximated B-Spline path")
    # plot_arrow(rax, ray, heading)

    # rix, riy, heading, curvature = interpolate_b_spline_path(
    #     way_point_x, way_point_y, n_course_point)
    # plt.plot(way_point_x, way_point_y, '-og', label="way points")
    # plt.plot(rix, riy, '-b', label="Interpolated B-Spline path")
    # plot_arrow(rix, riy, heading)
    # plot_curvature(rix, riy, heading, curvature)

    plt.grid(True)
    plt.legend()
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
