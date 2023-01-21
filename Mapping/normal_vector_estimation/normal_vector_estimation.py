import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.proj3d import proj_transform
from mpl_toolkits.mplot3d import art3d, Axes3D


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


setattr(Axes3D, 'arrow3D', _arrow3D)


def set_equal_axis(ax, x_lims, y_lims, z_lims):
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


def plot_triangle(p1, p2, p3, ax):
    ax.add_collection3d(art3d.Poly3DCollection([[p1, p2, p3]], color='b'))


def plot_3d_vector_arrow(ax, p1, p2):
    ax.arrow3D(p1[0], p1[1], p1[2],
               p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2],
               mutation_scale=20,
               arrowstyle="-|>",
               )


def calc_normal_vector(p1, p2, p3):
    """Calculate normal vector of triangle

    Args:
        p1 (np.array): 3D point
        p2 (np.array): 3D point
        p3 (np.array): 3D point

    Returns:
        np.array: normal vector
    """
    # calculate two vectors of triangle
    v1 = p2 - p1
    v2 = p3 - p1

    # calculate normal vector
    normal_vector = np.cross(v1, v2)

    # normalize vector
    normal_vector = normal_vector / np.linalg.norm(normal_vector)

    return normal_vector


def main():
    p1 = np.array([0.0, 0.0, 1.0])
    p2 = np.array([1.0, 1.0, 0.0])
    p3 = np.array([0.0, 1.0, 0.0])

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    set_equal_axis(ax, [0.0, 2.5], [0.0, 2.5], [0.0, 3.0])

    plot_triangle(p1, p2, p3, ax)
    center = np.mean([p1, p2, p3], axis=0)
    ax.plot(center[0], center[1], center[2], "ro")

    normal_vector = calc_normal_vector(p1, p2, p3)
    print(f"{center=}")
    print(f"{normal_vector=}")
    plot_3d_vector_arrow(ax, center, center + normal_vector)

    plt.show()


if __name__ == '__main__':
    main()
