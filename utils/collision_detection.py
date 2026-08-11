"""
    This module contains utility functions for collision detection
"""

from enum import Enum
import numpy as np


class PointPosition(Enum):
    LEFT = -1
    ON_LINE = 0
    RIGHT = 1


def cal_point_orientation_to_line(point, line_start, line_end):
    """
    Calculate the orientation of a point with respect to a line segment.

    Args:
        point (tuple): The point to check (x, y).
        line_start (tuple): The start point of the line segment (x1, y1).
        line_end (tuple): The end point of the line segment (x2, y2).

    Returns:
        PointPosition: The position of the point relative to the line segment.
    """
    x1, y1 = line_start
    x2, y2 = line_end
    x0, y0 = point

    # Calculate the cross-product
    cross_product = (x2 - x1) * (y0 - y1) - (y2 - y1) * (x0 - x1)

    if cross_product > 0:
        return PointPosition.LEFT
    elif cross_product < 0:
        return PointPosition.RIGHT
    else:
        return PointPosition.ON_LINE
