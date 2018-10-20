import sys
from unittest import TestCase

sys.path.append("./PathPlanning/FrenetOptimalTrajectory/")

from PathPlanning.FrenetOptimalTrajectory import frenet_optimal_trajectory as m


print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
