import os
import sys
from unittest import TestCase

sys.path.append(os.path.dirname(__file__) + "/../")
try:
    from PathPlanning.DynamicWindowApproach import dynamic_window_approach as m
except ImportError:
    raise

print(__file__)


class TestDynamicWindowApproach(TestCase):
    def test_main1(self):
        m.show_animation = False
        m.main(gx=1.0, gy=1.0)

    def test_main2(self):
        m.show_animation = False
        m.main(gx=1.0, gy=1.0, robot_type=m.RobotType.rectangle)


if __name__ == '__main__':  # pragma: no cover
    test = TestDynamicWindowApproach()
    test.test_main1()
    test.test_main2()
