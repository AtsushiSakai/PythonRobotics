from unittest import TestCase

import sys
sys.path.append("./PathPlanning/RRTStarDubins/")

from PathPlanning.RRTStarDubins import rrt_star_dubins as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
