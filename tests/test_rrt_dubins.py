from unittest import TestCase
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../PathPlanning/RRTDubins/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../PathPlanning/DubinsPath/")

try:
    import rrt_dubins as m
except:
    raise


print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
