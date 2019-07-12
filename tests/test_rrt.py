from unittest import TestCase

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
try:
    from PathPlanning.RRT import simple_rrt as m
    from PathPlanning.RRT import rrt_with_pathsmoothing as m1
except:
    raise


print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main(gx=1.0, gy=1.0)

    def test2(self):
        m1.show_animation = False
        m1.main()


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
    test.test2()
