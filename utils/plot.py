"""
Matplotlib based plotting utilities
"""
import math
import matplotlib.pyplot as plt
import numpy as np


def plot_arrow(x, y, yaw, arrow_length=1.0,
               origin_point_plot_style="xr",
               head_width=0.1, fc="r", ec="k", **kwargs):
    """
    Plot an arrow or arrows based on 2D state (x, y, yaw)

    All optional settings of matplotlib.pyplot.arrow can be used.
    - matplotlib.pyplot.arrow:
    https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.arrow.html

    Parameters
    ----------
    x : a float or array_like
        a value or a list of arrow origin x position.
    y : a float or array_like
        a value or a list of arrow origin y position.
    yaw : a float or array_like
        a value or a list of arrow yaw angle (orientation).
    arrow_length : a float (optional)
        arrow length. default is 1.0
    origin_point_plot_style : str (optional)
        origin point plot style. If None, not plotting.
    head_width : a float (optional)
        arrow head width. default is 0.1
    fc : string (optional)
        face color
    ec : string (optional)
        edge color
    """
    if not isinstance(x, float):
        for (i_x, i_y, i_yaw) in zip(x, y, yaw):
            plot_arrow(i_x, i_y, i_yaw, head_width=head_width,
                       fc=fc, ec=ec, **kwargs)
    else:
        plt.arrow(x, y,
                  arrow_length * math.cos(yaw),
                  arrow_length * math.sin(yaw),
                  head_width=head_width,
                  fc=fc, ec=ec,
                  **kwargs)
        if origin_point_plot_style is not None:
            plt.plot(x, y, origin_point_plot_style)


def plot_curvature(x_list, y_list, heading, curvature,
                   k=0.01, c="-c", label="Curvature"):
    """
    Plot curvature on 2D path. This plot is a line from the original path,
    the lateral distance from the original path shows curvature magnitude.
    Left turning shows right side plot, right turning shows left side plot.
    For straight path, the curvature plot will be on the path, because
    curvature is 0 on the straight path.

    Parameters
    ----------
    x_list : x position list of the path
    y_list : y position list of the path
    heading : heading list of the path
    curvature : curvature list of the path
    k : curvature scale factor to calculate distance from the original path
    c : color of the plot
    label : label of the plot
    """
    cx = [x + d * k * np.cos(yaw - np.pi / 2.0) for x, y, yaw, d in
          zip(x_list, y_list, heading, curvature)]
    cy = [y + d * k * np.sin(yaw - np.pi / 2.0) for x, y, yaw, d in
          zip(x_list, y_list, heading, curvature)]

    plt.plot(cx, cy, c, label=label)
    for ix, iy, icx, icy in zip(x_list, y_list, cx, cy):
        plt.plot([ix, icx], [iy, icy], c)
