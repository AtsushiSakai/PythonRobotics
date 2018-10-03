from unittest import TestCase

import sys
sys.path.append("./PathPlanning/LQRRRTStar/")

from PathPlanning.LQRRRTStar import lqr_rrt_star as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
