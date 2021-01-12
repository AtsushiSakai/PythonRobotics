import conftest
from PathPlanning.ClosedLoopRRTStar import closed_loop_rrt_star_car as m
import random


def test_1():
    random.seed(12345)
    m.show_animation = False
    m.main(gx=1.0, gy=0.0, gyaw=0.0, max_iter=5)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
