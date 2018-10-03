from unittest import TestCase

from SLAM.FastSLAM2 import fast_slam2 as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
