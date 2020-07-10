from unittest import TestCase
import sys
import os
import random
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
sys.path.append(os.path.dirname(os.path.abspath(__file__))
                + "/../PathPlanning/LQRRRTStar/")
try:
    from PathPlanning.LQRRRTStar import lqr_rrt_star as m
except ImportError:
    raise

print(__file__)

random.seed(12345)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main(maxIter=5)


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
