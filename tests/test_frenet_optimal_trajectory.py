from unittest import TestCase

import sys
import os
sys.path.append("./PathPlanning/FrenetOptimalTrajectory/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
try:
    from PathPlanning.FrenetOptimalTrajectory import frenet_optimal_trajectory as m
except:
    raise


print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.SIM_LOOP = 5
        m.main()


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
