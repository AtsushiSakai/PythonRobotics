from unittest import TestCase
from PathPlanning.DubinsPath import dubins_path_planning
import math


class Test(TestCase):

    def test1(self):
        start_x = 1.0  # [m]
        start_y = 1.0  # [m]
        start_yaw = math.radians(45.0)  # [rad]

        end_x = -3.0  # [m]
        end_y = -3.0  # [m]
        end_yaw = math.radians(-45.0)  # [rad]

        curvature = 1.0

        px, py, pyaw, mode, clen = dubins_path_planning.dubins_path_planning(
            start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)

        assert(abs(px[-1] - end_x) <= 0.1)
        assert(abs(py[-1] - end_y) <= 0.1)
        assert(abs(pyaw[-1] - end_yaw) <= 0.1)
