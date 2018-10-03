from unittest import TestCase

from SLAM.FastSLAM1 import fast_slam1 as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
