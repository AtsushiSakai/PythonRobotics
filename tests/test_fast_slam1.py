import conftest
from SLAM.FastSLAM1 import fast_slam1 as m


def test1():
    m.show_animation = False
    m.SIM_TIME = 3.0
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
