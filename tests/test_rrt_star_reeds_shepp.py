from unittest import TestCase

import sys
sys.path.append("./PathPlanning/RRTStarReedsShepp/")
sys.path.append("./PathPlanning/ReedsSheppPath/")

from PathPlanning.RRTStarReedsShepp import rrt_star_reeds_shepp as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
