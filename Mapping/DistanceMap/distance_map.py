"""
Distance Map

author: Wang Zheng (@Aglargil)

Reference:
- [Distance Map](https://cs.brown.edu/people/pfelzens/papers/dt-final.pdf)
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt  # Change: Explicit import

INF = 1e20
ENABLE_PLOT = True


def compute_sdf_scipy(obstacles: np.ndarray) -> np.ndarray:
    """
    Compute the signed distance field (SDF) from a boolean field using scipy.

    Parameters
    ----------
    obstacles : np.ndarray
        A 2D boolean array where '1' represents obstacles and '0' represents free space.

    Returns
    -------
    np.ndarray
        A 2D array representing the signed distance field.
    """
    # distance_transform_edt use '0' as obstacles, so we need to convert the obstacles to '0'
    a = distance_transform_edt(obstacles == 0)
    b = distance_transform_edt(obstacles == 1)
    
    # Fix: Changed (+) to (-) to correctly calculate Signed Distance
    return a - b


def compute_udf_scipy(obstacles: np.ndarray) -> np.ndarray:
    """
    Compute the unsigned distance field (UDF) from a boolean field using scipy.
    """
    return distance_transform_edt(obstacles == 0)


def compute_sdf(obstacles: np.ndarray) -> np.ndarray:
    """
    Compute the signed distance field (SDF) from a boolean field.
    """
    a = compute_udf(obstacles)
    b = compute_udf(obstacles == 0)
    return a - b


def compute_udf(obstacles: np.ndarray) -> np.ndarray:
    """
    Compute the unsigned distance field (UDF) from a boolean field.
    """
    edt = obstacles.copy().astype(float) # Ensure float for INF values
    
    if not np.all(np.isin(obstacles, [0, 1])):
        raise ValueError("Input array should only contain 0 and 1")
    
    # Fix: Initialize free space to INF, not 0. 
    # If set to 0, the minimization step in DT will not propagate distances.
    edt = np.where(edt == 0, INF, edt)
    edt = np.where(edt == 1, 0, edt)

    # Pass 1: Rows
    for row in range(len(edt)):
        dt(edt[row])

    edt = edt.T

    # Pass 2: Columns
    for row in range(len(edt)):
        dt(edt[row])

    # Fix: Added missing transpose to return array to original orientation
    edt = edt.T

    return np.sqrt(edt)


def dt(d: np.ndarray):
    """
    Compute 1D distance transform under the squared Euclidean distance
    Modifies input array 'd' in-place.
    """
    n = len(d)
    # Fix: Arrays v and z must be size n+1 to handle boundary conditions in the loop
    # Change: Enforce integer type for index array 'v'
    v = np.zeros(n + 1, dtype=int)
    z = np.zeros(n + 1)

    k = 0
    v[0] = 0
    z[0] = -INF
    z[1] = INF

    for q in range(1, n):
        # Calculate intersection of parabolas
        s = ((d[q] + q * q) - (d[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
        
        while s <= z[k]:
            k = k - 1
            s = ((d[q] + q * q) - (d[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
        
        k = k + 1
        v[k] = q
        z[k] = s
        z[k + 1] = INF

    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k = k + 1
        
        dx = q - v[k]
        d[q] = dx * dx + d[v[k]]


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
        # Fix: Typo 'sublots' -> 'subplots'
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
