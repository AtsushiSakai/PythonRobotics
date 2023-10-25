# Copyright (c) 2020 Jeff Irion and contributors
#
# This file originated from the `graphslam` package:
#
#   https://github.com/JeffLIrion/python-graphslam

r"""Representation of a pose in :math:`SE(2)`.

"""

import math
import numpy as np

from ..util import neg_pi_to_pi


class PoseSE2(np.ndarray):
    r"""A representation of a pose in :math:`SE(2)`.

    Parameters
    ----------
    position : np.ndarray, list
        The position in :math:`\mathbb{R}^2`
    orientation : float
        The angle of the pose (in radians)

    """

    def __new__(cls, position, orientation):
        obj = np.array([position[0], position[1], neg_pi_to_pi(orientation)],
                       dtype=float).view(cls)
        return obj

    # pylint: disable=arguments-differ
    def copy(self):
        """Return a copy of the pose.

        Returns
        -------
        PoseSE2
            A copy of the pose

        """
        return PoseSE2(self[:2], self[2])

    def to_array(self):
        """Return the pose as a numpy array.

        Returns
        -------
        np.ndarray
            The pose as a numpy array

        """
        return np.array(self)

    def to_compact(self):
        """Return the pose as a compact numpy array.

        Returns
        -------
        np.ndarray
            The pose as a compact numpy array

        """
        return np.array(self)

    def to_matrix(self):
        """Return the pose as an :math:`SE(2)` matrix.

        Returns
        -------
        np.ndarray
            The pose as an :math:`SE(2)` matrix

        """
        return np.array([[np.cos(self[2]), -np.sin(self[2]), self[0]],
                         [np.sin(self[2]), np.cos(self[2]), self[1]],
                         [0., 0., 1.]], dtype=float)

    @classmethod
    def from_matrix(cls, matrix):
        """Return the pose as an :math:`SE(2)` matrix.

        Parameters
        ----------
        matrix : np.ndarray
            The :math:`SE(2)` matrix that will be converted to a `PoseSE2` instance

        Returns
        -------
        PoseSE2
            The matrix as a `PoseSE2` object

        """
        return cls([matrix[0, 2], matrix[1, 2]],
                   math.atan2(matrix[1, 0], matrix[0, 0]))

    # ======================================================================= #
    #                                                                         #
    #                                Properties                               #
    #                                                                         #
    # ======================================================================= #
    @property
    def position(self):
        """Return the pose's position.

        Returns
        -------
        np.ndarray
            The position portion of the pose

        """
        return np.array(self[:2])

    @property
    def orientation(self):
        """Return the pose's orientation.

        Returns
        -------
        float
            The angle of the pose

        """
        return self[2]

    @property
    def inverse(self):
        """Return the pose's inverse.

        Returns
        -------
        PoseSE2
            The pose's inverse

        """
        return PoseSE2([-self[0] * np.cos(self[2]) - self[1] * np.sin(self[2]),
                        self[0] * np.sin(self[2]) - self[1] * np.cos(self[2])],
                        -self[2])

    # ======================================================================= #
    #                                                                         #
    #                              Magic Methods                              #
    #                                                                         #
    # ======================================================================= #
    def __add__(self, other):
        r"""Add poses (i.e., pose composition): :math:`p_1 \oplus p_2`.

        Parameters
        ----------
        other : PoseSE2
            The other pose

        Returns
        -------
        PoseSE2
            The result of pose composition

        """
        return PoseSE2(
            [self[0] + other[0] * np.cos(self[2]) - other[1] * np.sin(self[2]),
             self[1] + other[0] * np.sin(self[2]) + other[1] * np.cos(self[2])
             ], neg_pi_to_pi(self[2] + other[2]))

    def __sub__(self, other):
        r"""Subtract poses (i.e., inverse pose composition): :math:`p_1 \ominus p_2`.

        Parameters
        ----------
        other : PoseSE2
            The other pose

        Returns
        -------
        PoseSE2
            The result of inverse pose composition

        """
        return PoseSE2([(self[0] - other[0]) * np.cos(other[2]) + (
                    self[1] - other[1]) * np.sin(other[2]),
                        (other[0] - self[0]) * np.sin(other[2]) + (
                                    self[1] - other[1]) * np.cos(other[2])],
                       neg_pi_to_pi(self[2] - other[2]))
