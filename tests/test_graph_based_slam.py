from unittest import TestCase

from SLAM.GraphBasedSLAM import graph_based_slam as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
