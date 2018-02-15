from unittest import TestCase

import sys
sys.path.append("./PathPlanning/ClosedLoopRRTStar/")
sys.path.append("./PathPlanning/ReedsSheppPath/")

from PathPlanning.ClosedLoopRRTStar import closed_loop_rrt_star_car as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
