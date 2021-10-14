# Copyright (c) 2020 Jeff Irion and contributors
#
# This file originated from the `graphslam` package:
#
#   https://github.com/JeffLIrion/python-graphslam

r"""A class for odometry edges.

"""


import numpy as np
import matplotlib.pyplot as plt


#: The difference that will be used for numerical differentiation
EPSILON = 1e-6


class EdgeOdometry:
    r"""A class for representing odometry edges in Graph SLAM.

    Parameters
    ----------
    vertices : list[graphslam.vertex.Vertex]
        A list of the vertices constrained by the edge
    information : np.ndarray
        The information matrix :math:`\Omega_j` associated with the edge
    estimate : graphslam.pose.se2.PoseSE2
        The expected measurement :math:`\mathbf{z}_j`

    Attributes
    ----------
    vertices : list[graphslam.vertex.Vertex]
        A list of the vertices constrained by the edge
    information : np.ndarray
        The information matrix :math:`\Omega_j` associated with the edge
    estimate : PoseSE2
        The expected measurement :math:`\mathbf{z}_j`

    """
    def __init__(self, vertex_ids, information, estimate, vertices=None):
        self.vertex_ids = vertex_ids
        self.information = information
        self.estimate = estimate
        self.vertices = vertices

    def calc_error(self):
        r"""Calculate the error for the edge: :math:`\mathbf{e}_j \in \mathbb{R}^\bullet`.

        .. math::

           \mathbf{e}_j = \mathbf{z}_j - (p_2 \ominus p_1)


        Returns
        -------
        np.ndarray
            The error for the edge

        """
        return (self.estimate - (self.vertices[1].pose - self.vertices[0].pose)).to_compact()

    def calc_chi2(self):
        r"""Calculate the :math:`\chi^2` error for the edge.

        .. math::

           \mathbf{e}_j^T \Omega_j \mathbf{e}_j


        Returns
        -------
        float
            The :math:`\chi^2` error for the edge

        """
        err = self.calc_error()

        return np.dot(np.dot(np.transpose(err), self.information), err)

    def calc_chi2_gradient_hessian(self):
        r"""Calculate the edge's contributions to the graph's :math:`\chi^2` error, gradient (:math:`\mathbf{b}`), and Hessian (:math:`H`).

        Returns
        -------
        float
            The :math:`\chi^2` error for the edge
        dict
            The edge's contribution(s) to the gradient
        dict
            The edge's contribution(s) to the Hessian

        """
        chi2 = self.calc_chi2()

        err = self.calc_error()

        jacobians = self.calc_jacobians()

        return chi2, {v.index: np.dot(np.dot(np.transpose(err), self.information), jacobian) for v, jacobian in zip(self.vertices, jacobians)}, {(self.vertices[i].index, self.vertices[j].index): np.dot(np.dot(np.transpose(jacobians[i]), self.information), jacobians[j]) for i in range(len(jacobians)) for j in range(i, len(jacobians))}

    def calc_jacobians(self):
        r"""Calculate the Jacobian of the edge's error with respect to each constrained pose.

        .. math::

           \frac{\partial}{\partial \Delta \mathbf{x}^k} \left[ \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k) \right]


        Returns
        -------
        list[np.ndarray]
            The Jacobian matrices for the edge with respect to each constrained pose

        """
        err = self.calc_error()

        # The dimensionality of the compact pose representation
        dim = len(self.vertices[0].pose.to_compact())

        return [self._calc_jacobian(err, dim, i) for i in range(len(self.vertices))]

    def _calc_jacobian(self, err, dim, vertex_index):
        r"""Calculate the Jacobian of the edge with respect to the specified vertex's pose.

        Parameters
        ----------
        err : np.ndarray
            The current error for the edge (see :meth:`EdgeOdometry.calc_error`)
        dim : int
            The dimensionality of the compact pose representation
        vertex_index : int
            The index of the vertex (pose) for which we are computing the Jacobian

        Returns
        -------
        np.ndarray
            The Jacobian of the edge with respect to the specified vertex's pose

        """
        jacobian = np.zeros(err.shape + (dim,))
        p0 = self.vertices[vertex_index].pose.copy()

        for d in range(dim):
            # update the pose
            delta_pose = np.zeros(dim)
            delta_pose[d] = EPSILON
            self.vertices[vertex_index].pose += delta_pose

            # compute the numerical derivative
            jacobian[:, d] = (self.calc_error() - err) / EPSILON

            # restore the pose
            self.vertices[vertex_index].pose = p0.copy()

        return jacobian

    def to_g2o(self):
        """Export the edge to the .g2o format.

        Returns
        -------
        str
            The edge in .g2o format

        """
        return "EDGE_SE2 {} {} {} {} {} ".format(self.vertex_ids[0], self.vertex_ids[1], self.estimate[0], self.estimate[1], self.estimate[2]) + " ".join([str(x) for x in self.information[np.triu_indices(3, 0)]]) + "\n"

    def plot(self, color='b'):
        """Plot the edge.

        Parameters
        ----------
        color : str
            The color that will be used to plot the edge

        """
        xy = np.array([v.pose.position for v in self.vertices])
        plt.plot(xy[:, 0], xy[:, 1], color=color)
