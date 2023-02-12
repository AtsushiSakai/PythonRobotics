import numpy as np
from matplotlib import pyplot as plt

from utils.plot import plot_3d_vector_arrow, plot_triangle, set_equal_3d_axis


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

    set_equal_3d_axis(ax, [0.0, 2.5], [0.0, 2.5], [0.0, 3.0])

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
