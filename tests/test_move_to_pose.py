import conftest  # Add root path to sys.path
from PathTracking.move_to_pose import move_to_pose as m


def test_1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
