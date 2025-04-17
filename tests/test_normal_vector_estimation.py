import conftest  # Add root path to sys.path
from Mapping.normal_vector_estimation import normal_vector_estimation as m
import random

random.seed(12345)


def test_1():
    m.show_animation = False
    m.main1()


def test_2():
    m.show_animation = False
    m.main2()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
