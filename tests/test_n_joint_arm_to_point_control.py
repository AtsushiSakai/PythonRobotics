from unittest import TestCase

import sys
sys.path.append("./ArmNavigation/n_joint_arm_to_point_control/")

from ArmNavigation.n_joint_arm_to_point_control import n_joint_arm_to_point_control as m
print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.animation()
