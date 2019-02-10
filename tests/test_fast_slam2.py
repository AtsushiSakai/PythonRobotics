from unittest import TestCase
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
try:
    from SLAM.FastSLAM2 import fast_slam2 as m
except:
    raise


print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.SIM_TIME = 3.0
        m.main()


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
