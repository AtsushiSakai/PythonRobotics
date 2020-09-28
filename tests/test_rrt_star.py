import os
import sys
from unittest import TestCase

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../PathPlanning/RRTStar/")

try:
    import rrt_star as m
except ImportError:
    raise


print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()

    def test_no_obstacle(self):
        obstacle_list = []

        # Set Initial parameters
        rrt_star = m.RRTStar(start=[0, 0],
                             goal=[6, 10],
                             rand_area=[-2, 15],
                             obstacle_list=obstacle_list)
        path = rrt_star.planning(animation=False)
        assert path is not None

if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
    test.test_no_obstacle()
