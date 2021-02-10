# Copyright (c) 2020 Jeff Irion and contributors
#
# This file originated from the `graphslam` package:
#
#   https://github.com/JeffLIrion/python-graphslam

r"""A ``Graph`` class that stores the edges and vertices required for Graph SLAM.

"""

import warnings
from collections import defaultdict
from functools import reduce

import matplotlib.pyplot as plt
import numpy as np
from scipy.sparse import SparseEfficiencyWarning, lil_matrix
from scipy.sparse.linalg import spsolve

warnings.simplefilter("ignore", SparseEfficiencyWarning)
warnings.filterwarnings("ignore", category=SparseEfficiencyWarning)


# pylint: disable=too-few-public-methods
class _Chi2GradientHessian:
    r"""A class that is used to aggregate the :math:`\chi^2` error, gradient, and Hessian.

    Parameters
    ----------
    dim : int
        The compact dimensionality of the poses

    Attributes
    ----------
    chi2 : float
        The :math:`\chi^2` error
    dim : int
        The compact dimensionality of the poses
    gradient : defaultdict
        The contributions to the gradient vector
    hessian : defaultdict
        The contributions to the Hessian matrix

    """

    def __init__(self, dim):
        self.chi2 = 0.
        self.dim = dim
        self.gradient = defaultdict(lambda: np.zeros(dim))
        self.hessian = defaultdict(lambda: np.zeros((dim, dim)))

    @staticmethod
    def update(chi2_grad_hess, incoming):
        r"""Update the :math:`\chi^2` error and the gradient and Hessian dictionaries.

        Parameters
        ----------
        chi2_grad_hess : _Chi2GradientHessian
            The ``_Chi2GradientHessian`` that will be updated
        incoming : tuple

        """
        chi2_grad_hess.chi2 += incoming[0]

        for idx, contrib in incoming[1].items():
            chi2_grad_hess.gradient[idx] += contrib

        for (idx1, idx2), contrib in incoming[2].items():
            if idx1 <= idx2:
                chi2_grad_hess.hessian[idx1, idx2] += contrib
            else:
                chi2_grad_hess.hessian[idx2, idx1] += np.transpose(contrib)

        return chi2_grad_hess


class Graph(object):
    r"""A graph that will be optimized via Graph SLAM.

    Parameters
    ----------
    edges : list[graphslam.edge.edge_odometry.EdgeOdometry]
        A list of the vertices in the graph
    vertices : list[graphslam.vertex.Vertex]
        A list of the vertices in the graph

    Attributes
    ----------
    _chi2 : float, None
        The current :math:`\chi^2` error, or ``None`` if it has not yet been computed
    _edges : list[graphslam.edge.edge_odometry.EdgeOdometry]
        A list of the edges (i.e., constraints) in the graph
    _gradient : numpy.ndarray, None
        The gradient :math:`\mathbf{b}` of the :math:`\chi^2` error, or ``None`` if it has not yet been computed
    _hessian : scipy.sparse.lil_matrix, None
        The Hessian matrix :math:`H`, or ``None`` if it has not yet been computed
    _vertices : list[graphslam.vertex.Vertex]
        A list of the vertices in the graph

    """

    def __init__(self, edges, vertices):
        # The vertices and edges lists
        self._edges = edges
        self._vertices = vertices

        # The chi^2 error, gradient, and Hessian
        self._chi2 = None
        self._gradient = None
        self._hessian = None

        self._link_edges()

    def _link_edges(self):
        """Fill in the ``vertices`` attributes for the graph's edges.

        """
        index_id_dict = {i: v.id for i, v in enumerate(self._vertices)}
        id_index_dict = {v_id: v_index for v_index, v_id in
                         index_id_dict.items()}

        # Fill in the vertices' `index` attribute
        for v in self._vertices:
            v.index = id_index_dict[v.id]

        for e in self._edges:
            e.vertices = [self._vertices[id_index_dict[v_id]] for v_id in
                          e.vertex_ids]

    def calc_chi2(self):
        r"""Calculate the :math:`\chi^2` error for the ``Graph``.

        Returns
        -------
        float
            The :math:`\chi^2` error

        """
        self._chi2 = sum((e.calc_chi2() for e in self._edges))
        return self._chi2

    def _calc_chi2_gradient_hessian(self):
        r"""Calculate the :math:`\chi^2` error, the gradient :math:`\mathbf{b}`, and the Hessian :math:`H`.

        """
        n = len(self._vertices)
        dim = len(self._vertices[0].pose.to_compact())
        chi2_gradient_hessian = reduce(_Chi2GradientHessian.update,
                                       (e.calc_chi2_gradient_hessian()
                                        for e in self._edges),
                                       _Chi2GradientHessian(dim))

        self._chi2 = chi2_gradient_hessian.chi2

        # Fill in the gradient vector
        self._gradient = np.zeros(n * dim, dtype=float)
        for idx, cont in chi2_gradient_hessian.gradient.items():
            self._gradient[idx * dim: (idx + 1) * dim] += cont

        # Fill in the Hessian matrix
        self._hessian = lil_matrix((n * dim, n * dim), dtype=float)
        for (row_idx, col_idx), cont in chi2_gradient_hessian.hessian.items():
            x_start = row_idx * dim
            x_end = (row_idx + 1) * dim
            y_start = col_idx * dim
            y_end = (col_idx + 1) * dim
            self._hessian[x_start:x_end, y_start:y_end] = cont

            if row_idx != col_idx:
                x_start = col_idx * dim
                x_end = (col_idx + 1) * dim
                y_start = row_idx * dim
                y_end = (row_idx + 1) * dim
                self._hessian[x_start:x_end, y_start:y_end] = \
                    np.transpose(cont)

    def optimize(self, tol=1e-4, max_iter=20, fix_first_pose=True):
        r"""Optimize the :math:`\chi^2` error for the ``Graph``.

        Parameters
        ----------
        tol : float
            If the relative decrease in the :math:`\chi^2` error between iterations is less than ``tol``, we will stop
        max_iter : int
            The maximum number of iterations
        fix_first_pose : bool
            If ``True``, we will fix the first pose

        """
        n = len(self._vertices)
        dim = len(self._vertices[0].pose.to_compact())

        # Previous iteration's chi^2 error
        chi2_prev = -1.

        # For displaying the optimization progress
        print("\nIteration                chi^2        rel. change")
        print("---------                -----        -----------")

        for i in range(max_iter):
            self._calc_chi2_gradient_hessian()

            # Check for convergence (from the previous iteration); this avoids having to calculate chi^2 twice
            if i > 0:
                rel_diff = (chi2_prev - self._chi2) / (
                            chi2_prev + np.finfo(float).eps)
                print(
                    "{:9d} {:20.4f} {:18.6f}".format(i, self._chi2, -rel_diff))
                if self._chi2 < chi2_prev and rel_diff < tol:
                    return
            else:
                print("{:9d} {:20.4f}".format(i, self._chi2))

            # Update the previous iteration's chi^2 error
            chi2_prev = self._chi2

            # Hold the first pose fixed
            if fix_first_pose:
                self._hessian[:dim, :] = 0.
                self._hessian[:, :dim] = 0.
                self._hessian[:dim, :dim] += np.eye(dim)
                self._gradient[:dim] = 0.

            # Solve for the updates
            dx = spsolve(self._hessian, -self._gradient)

            # Apply the updates
            for v, dx_i in zip(self._vertices, np.split(dx, n)):
                v.pose += dx_i

        # If we reached the maximum number of iterations, print out the final iteration's results
        self.calc_chi2()
        rel_diff = (chi2_prev - self._chi2) / (chi2_prev + np.finfo(float).eps)
        print("{:9d} {:20.4f} {:18.6f}".format(
            max_iter, self._chi2, -rel_diff))

    def to_g2o(self, outfile):
        """Save the graph in .g2o format.

        Parameters
        ----------
        outfile : str
            The path where the graph will be saved

        """
        with open(outfile, 'w') as f:
            for v in self._vertices:
                f.write(v.to_g2o())

            for e in self._edges:
                f.write(e.to_g2o())

    def plot(self, vertex_color='r', vertex_marker='o', vertex_markersize=3,
             edge_color='b', title=None):
        """Plot the graph.

        Parameters
        ----------
        vertex_color : str
            The color that will be used to plot the vertices
        vertex_marker : str
            The marker that will be used to plot the vertices
        vertex_markersize : int
            The size of the plotted vertices
        edge_color : str
            The color that will be used to plot the edges
        title : str, None
            The title that will be used for the plot

        """
        plt.figure()

        for e in self._edges:
            e.plot(edge_color)

        for v in self._vertices:
            v.plot(vertex_color, vertex_marker, vertex_markersize)

        if title:
            plt.title(title)

        plt.show()
