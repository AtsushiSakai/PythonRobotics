import conftest  # Add root path to sys.path
from ArmNavigation.n_joint_arm_to_point_control\
    import n_joint_arm_to_point_control as m
import random

random.seed(12345)


def test1():
    m.show_animation = False
    m.animation()
