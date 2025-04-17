import conftest
from SLAM.GraphBasedSLAM import graph_based_slam as m


def test_1():
    m.show_animation = False
    m.SIM_TIME = 20.0
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
