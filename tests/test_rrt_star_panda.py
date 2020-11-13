import os
import sys
from unittest import TestCase

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../ArmNavigation/panda_robot_to_point_control/")

try:
    import PathPlanner as m
except ImportError:
    raise


print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()

if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
