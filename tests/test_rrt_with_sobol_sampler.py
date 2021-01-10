import os
import sys
import random
from unittest import TestCase

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
sys.path.append(os.path.dirname(
    os.path.abspath(__file__)) + "/../PathPlanning/RRT")

try:
    from PathPlanning.RRT import rrt_with_sobol_sampler as m
except ImportError:
    raise


print(__file__)

random.seed(12345)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main(gx=1.0, gy=1.0)


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
