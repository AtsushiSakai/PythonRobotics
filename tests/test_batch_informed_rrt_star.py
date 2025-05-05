import random

import conftest
from PathPlanning.BatchInformedRRTStar import batch_informed_rrt_star as m


def test_1():
    m.show_animation = False
    random.seed(12345)
    m.main(maxIter=10)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
