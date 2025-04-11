import conftest  # Add root path to sys.path
from PathPlanning.RRT import rrt_with_sobol_sampler as m
import random

random.seed(12345)


def test1():
    m.show_animation = False
    m.main(gx=1.0, gy=1.0)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
