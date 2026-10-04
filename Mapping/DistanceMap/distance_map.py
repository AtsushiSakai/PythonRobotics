"""
Distance Map

author: Wang Zheng (@Aglargil)

Reference:

- [Distance Map]
(https://cs.brown.edu/people/pfelzens/papers/dt-final.pdf)
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy

INF = 1e20
ENABLE_PLOT = True


def compute_sdf_scipy(obstacles):
    """
    Compute the signed distance field (SDF) from a boolean field using scipy.
    This function has the same functionality as compute_sdf.
    However, by using scipy.ndimage.distance_transform_edt, it can compute much faster.

    Example: 500×500 map
    • compute_sdf: 3 sec
    • compute_sdf_scipy: 0.05 sec

    Parameters
    ----------
    obstacles : array_like
        A 2D boolean array where '1' represents obstacles and '0' represents free space.

    Returns
    -------
    array_like
        A 2D array representing the signed distance field, where positive values indicate distance
        to the nearest obstacle, and negative values indicate distance to the nearest free space.
    """
    # distance_transform_edt use '0' as obstacles, so we need to convert the obstacles to '0'
    a = scipy.ndimage.distance_transform_edt(obstacles == 0)
    b = scipy.ndimage.distance_transform_edt(obstacles == 1)
    return a - b


def compute_udf_scipy(obstacles):
    """
    Compute the unsigned distance field (UDF) from a boolean field using scipy.
    This function has the same functionality as compute_udf.
    However, by using scipy.ndimage.distance_transform_edt, it can compute much faster.

    Example: 500×500 map
    • compute_udf: 1.5 sec
    • compute_udf_scipy: 0.02 sec

    Parameters
    ----------
    obstacles : array_like
        A 2D boolean array where '1' represents obstacles and '0' represents free space.

    Returns
    -------
    array_like
        A 2D array of distances from the nearest obstacle, with the same dimensions as `bool_field`.
    """
    return scipy.ndimage.distance_transform_edt(obstacles == 0)


def compute_sdf(obstacles):
    """
    Compute the signed distance field (SDF) from a boolean field.

    Parameters
    ----------
    obstacles : array_like
        A 2D boolean array where '1' represents obstacles and '0' represents free space.

    Returns
    -------
    array_like
        A 2D array representing the signed distance field, where positive values indicate distance
        to the nearest obstacle, and negative values indicate distance to the nearest free space.
    """
    a = compute_udf(obstacles)
    b = compute_udf(obstacles == 0)
    return a - b


def compute_udf(obstacles):
    """
    Compute the unsigned distance field (UDF) from a boolean field.

    Parameters
    ----------
    obstacles : array_like
        A 2D boolean array where '1' represents obstacles and '0' represents free space.

    Returns
    -------
    array_like
        A 2D array of distances from the nearest obstacle, with the same dimensions as `bool_field`.
    """
    edt = obstacles.copy()
    if not np.all(np.isin(edt, [0, 1])):
        raise ValueError("Input array should only contain 0 and 1")
    edt = np.where(edt == 0, INF, edt)
    edt = np.where(edt == 1, 0, edt)
    for row in range(len(edt)):
        dt(edt[row])
    edt = edt.T
    for row in range(len(edt)):
        dt(edt[row])
    edt = edt.T
    return np.sqrt(edt)


def dt(d):
    """
    Compute 1D distance transform under the squared Euclidean distance

    Parameters
    ----------
    d : array_like
        Input array containing the distances.

    Returns:
    --------
    d : array_like
        The transformed array with computed distances.
    """
    v = np.zeros(len(d) + 1)
    z = np.zeros(len(d) + 1)
    k = 0
    v[0] = 0
    z[0] = -INF
    z[1] = INF
    for q in range(1, len(d)):
        s = ((d[q] + q * q) - (d[int(v[k])] + v[k] * v[k])) / (2 * q - 2 * v[k])
        while s <= z[k]:
            k = k - 1
            s = ((d[q] + q * q) - (d[int(v[k])] + v[k] * v[k])) / (2 * q - 2 * v[k])
        k = k + 1
        v[k] = q
        z[k] = s
        z[k + 1] = INF
    k = 0
    for q in range(len(d)):
        while z[k + 1] < q:
            k = k + 1
        dx = q - v[k]
        d[q] = dx * dx + d[int(v[k])]


def main():
    obstacles = np.array(
        [
            [1, 0, 0, 0, 0],
            [0, 1, 1, 1, 0],
            [0, 1, 1, 1, 0],
            [0, 0, 1, 1, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ]
    )

    # Compute the signed distance field
    sdf = compute_sdf(obstacles)
    udf = compute_udf(obstacles)

    if ENABLE_PLOT:
        _, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))

        obstacles_plot = ax1.imshow(obstacles, cmap="binary")
        ax1.set_title("Obstacles")
        ax1.set_xlabel("x")
        ax1.set_ylabel("y")
        plt.colorbar(obstacles_plot, ax=ax1)

        udf_plot = ax2.imshow(udf, cmap="viridis")
        ax2.set_title("Unsigned Distance Field")
        ax2.set_xlabel("x")
        ax2.set_ylabel("y")
        plt.colorbar(udf_plot, ax=ax2)

        sdf_plot = ax3.imshow(sdf, cmap="RdBu")
        ax3.set_title("Signed Distance Field")
        ax3.set_xlabel("x")
        ax3.set_ylabel("y")
        plt.colorbar(sdf_plot, ax=ax3)

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
