from unittest import TestCase
import sys
sys.path.append("./PathPlanning/RRTDubins/")
sys.path.append("./PathPlanning/DubinsPath/")

try:
    from PathPlanning.RRTDubins import rrt_dubins as m
    # from RRTDubins import rrt_dubins as m
except:
    raise


print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
