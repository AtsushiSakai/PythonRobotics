import conftest  # Add root path to sys.path
from ArmNavigation.two_joint_arm_to_point_control \
    import two_joint_arm_to_point_control as m


def test1():
    m.show_animation = False
    m.animation()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
