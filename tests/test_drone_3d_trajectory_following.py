from unittest import TestCase

import sys
sys.path.append("./AerialNavigation/drone_3d_trajectory_following/")

from AerialNavigation.drone_3d_trajectory_following import drone_3d_trajectory_following as m
print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
