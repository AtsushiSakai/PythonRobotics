import pytest
import numpy as np
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


def test_distance_is_independent_of_normal_scale():
    origin = np.array([0., 0., 3.])
    point = np.array([1., 2., 5.])
    for scale in (1., -1., 1e-10):
        normal = np.array([0., 0., scale])
        assert m.distance_to_plane(point, normal, origin) == pytest.approx(2.)
        assert m.distance_to_plane(origin, normal, origin) == 0.


if __name__ == '__main__':
    conftest.run_this_test(__file__)
