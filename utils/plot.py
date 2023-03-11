"""
Matplotlib based plotting utilities
"""
import math
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import art3d
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.proj3d import proj_transform
from mpl_toolkits.mplot3d import Axes3D


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


def plot_curvature(x_list, y_list, heading_list, curvature,
                   k=0.01, c="-c", label="Curvature"):
    """
    Plot curvature on 2D path. This plot is a line from the original path,
    the lateral distance from the original path shows curvature magnitude.
    Left turning shows right side plot, right turning shows left side plot.
    For straight path, the curvature plot will be on the path, because
    curvature is 0 on the straight path.

    Parameters
    ----------
    x_list : array_like
        x position list of the path
    y_list : array_like
        y position list of the path
    heading_list : array_like
        heading list of the path
    curvature : array_like
        curvature list of the path
    k : float
        curvature scale factor to calculate distance from the original path
    c : string
        color of the plot
    label : string
        label of the plot
    """
    cx = [x + d * k * np.cos(yaw - np.pi / 2.0) for x, y, yaw, d in
          zip(x_list, y_list, heading_list, curvature)]
    cy = [y + d * k * np.sin(yaw - np.pi / 2.0) for x, y, yaw, d in
          zip(x_list, y_list, heading_list, curvature)]

    plt.plot(cx, cy, c, label=label)
    for ix, iy, icx, icy in zip(x_list, y_list, cx, cy):
        plt.plot([ix, icx], [iy, icy], c)


class Arrow3D(FancyArrowPatch):

    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)

    def do_3d_projection(self, renderer=None):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))

        return np.min(zs)


def _arrow3D(ax, x, y, z, dx, dy, dz, *args, **kwargs):
    '''Add an 3d arrow to an `Axes3D` instance.'''
    arrow = Arrow3D(x, y, z, dx, dy, dz, *args, **kwargs)
    ax.add_artist(arrow)


def plot_3d_vector_arrow(ax, p1, p2):
    setattr(Axes3D, 'arrow3D', _arrow3D)
    ax.arrow3D(p1[0], p1[1], p1[2],
               p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2],
               mutation_scale=20,
               arrowstyle="-|>",
               )



def plot_triangle(p1, p2, p3, ax):
    ax.add_collection3d(art3d.Poly3DCollection([[p1, p2, p3]], color='b'))


def set_equal_3d_axis(ax, x_lims, y_lims, z_lims):
    """Helper function to set equal axis

    Args:
        ax (Axes3DSubplot): matplotlib 3D axis, created by
        `ax = fig.add_subplot(projection='3d')`
        x_lims (np.array): array containing min and max value of x
        y_lims (np.array): array containing min and max value of y
        z_lims (np.array): array containing min and max value of z
    """
    x_lims = np.asarray(x_lims)
    y_lims = np.asarray(y_lims)
    z_lims = np.asarray(z_lims)
    # compute max required range
    max_range = np.array([x_lims.max() - x_lims.min(),
                          y_lims.max() - y_lims.min(),
                          z_lims.max() - z_lims.min()]).max() / 2.0
    # compute mid-point along each axis
    mid_x = (x_lims.max() + x_lims.min()) * 0.5
    mid_y = (y_lims.max() + y_lims.min()) * 0.5
    mid_z = (z_lims.max() + z_lims.min()) * 0.5

    # set limits to axis
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
