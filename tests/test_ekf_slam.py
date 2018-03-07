from unittest import TestCase

from SLAM.EKFSLAM import ekf_slam as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
