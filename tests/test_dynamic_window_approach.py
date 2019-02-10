from unittest import TestCase

import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../")
try:
    from PathPlanning.DynamicWindowApproach import dynamic_window_approach as m
except:
    raise

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main(gx=1.0, gy=1.0)


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
