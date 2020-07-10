import os
import sys
import random
from unittest import TestCase

sys.path.append(os.path.dirname(__file__) + "/../ArmNavigation/n_joint_arm_to_point_control/")

import n_joint_arm_to_point_control as m

print(__file__)

random.seed(12345)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.animation()
