import conftest
from SLAM.ICPMatching import icp_matching as m


def test_1():
    m.show_animation = False
    m.main()


def test_2():
    m.show_animation = False
    m.main_3d_points()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
