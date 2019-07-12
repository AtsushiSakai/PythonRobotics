from unittest import TestCase
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../PathPlanning/ClosedLoopRRTStar/")
try:
    from PathPlanning.ClosedLoopRRTStar import closed_loop_rrt_star_car as m
except:
    raise


print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main(gx=1.0, gy=0.0, gyaw=0.0, maxIter=5)


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
