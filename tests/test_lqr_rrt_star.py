import conftest  # Add root path to sys.path
from PathPlanning.LQRRRTStar import lqr_rrt_star as m
import random

random.seed(12345)


def test1():
    m.show_animation = False
    m.main(maxIter=5)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
