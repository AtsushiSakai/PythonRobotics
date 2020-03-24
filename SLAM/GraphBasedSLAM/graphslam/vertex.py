# Copyright (c) 2020 Jeff Irion and contributors
#
# This file originated from the `graphslam` package:
#
#   https://github.com/JeffLIrion/python-graphslam

"""A ``Vertex`` class.

"""

import matplotlib.pyplot as plt


# pylint: disable=too-few-public-methods
class Vertex:
    """A class for representing a vertex in Graph SLAM.

    Parameters
    ----------
    vertex_id : int
        The vertex's unique ID
    pose : graphslam.pose.se2.PoseSE2
        The pose associated with the vertex
    vertex_index : int, None
        The vertex's index in the graph's ``vertices`` list

    Attributes
    ----------
    id : int
        The vertex's unique ID
    index : int, None
        The vertex's index in the graph's ``vertices`` list
    pose : graphslam.pose.se2.PoseSE2
        The pose associated with the vertex

    """
    def __init__(self, vertex_id, pose, vertex_index=None):
        self.id = vertex_id
        self.pose = pose
        self.index = vertex_index

    def to_g2o(self):
        """Export the vertex to the .g2o format.

        Returns
        -------
        str
            The vertex in .g2o format

        """
        return "VERTEX_SE2 {} {} {} {}\n".format(self.id, self.pose[0], self.pose[1], self.pose[2])

    def plot(self, color='r', marker='o', markersize=3):
        """Plot the vertex.

        Parameters
        ----------
        color : str
            The color that will be used to plot the vertex
        marker : str
            The marker that will be used to plot the vertex
        markersize : int
            The size of the plotted vertex

        """
        x, y = self.pose.position
        plt.plot(x, y, color=color, marker=marker, markersize=markersize)
