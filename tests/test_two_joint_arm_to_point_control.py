from unittest import TestCase

from ArmNavigation.two_joint_arm_to_point_control import two_joint_arm_to_point_control as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.animation()
