from unittest import TestCase

import sys
sys.path.append("./PathPlanning/RRTDubins/")

from PathPlanning.RRTDubins import rrt_dubins as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
