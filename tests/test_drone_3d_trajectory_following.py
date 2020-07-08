import os
import sys
from unittest import TestCase

sys.path.append(os.path.dirname(__file__) + "/../AerialNavigation/drone_3d_trajectory_following/")

import drone_3d_trajectory_following as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
