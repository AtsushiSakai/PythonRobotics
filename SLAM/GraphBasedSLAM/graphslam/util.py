# Copyright (c) 2020 Jeff Irion and contributors
#
# This file originated from the `graphslam` package:
#
#   https://github.com/JeffLIrion/python-graphslam

"""Utility functions used throughout the package.

"""

import numpy as np


TWO_PI = 2 * np.pi


def neg_pi_to_pi(angle):
    r"""Normalize ``angle`` to be in :math:`[-\pi, \pi)`.

    Parameters
    ----------
    angle : float
        An angle (in radians)

    Returns
    -------
    float
        The angle normalized to :math:`[-\pi, \pi)`

    """
    return (angle + np.pi) % (TWO_PI) - np.pi


def solve_for_edge_dimensionality(n):
    r"""Solve for the dimensionality of an edge.

    In a .g2o file, an edge is specified as ``<estimate> <information matrix>``, where only the upper triangular portion of the matrix is provided.

    This solves the problem:

    .. math::

       d + \frac{d (d + 1)}{2} = n

    Returns
    -------
    int
        The dimensionality of the edge

    """
    return int(round(np.sqrt(2 * n + 2.25) - 1.5))


def upper_triangular_matrix_to_full_matrix(arr, n):
    """Given an upper triangular matrix, return the full matrix.

    Parameters
    ----------
    arr : np.ndarray
        The upper triangular portion of the matrix
    n : int
        The size of the matrix

    Returns
    -------
    mat : np.ndarray
        The full matrix

    """
    triu0 = np.triu_indices(n, 0)
    tril1 = np.tril_indices(n, -1)

    mat = np.zeros((n, n), dtype=float)
    mat[triu0] = arr
    mat[tril1] = mat.T[tril1]

    return mat
