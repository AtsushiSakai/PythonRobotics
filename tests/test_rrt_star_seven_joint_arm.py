import conftest  # Add root path to sys.path
from ArmNavigation.rrt_star_seven_joint_arm_control \
    import rrt_star_seven_joint_arm_control as m


def test1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
