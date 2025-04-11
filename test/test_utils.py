import conftest  # Add root path to sys.path
from utils import angle
from numpy.testing import assert_allclose
import numpy as np


def test_rot_mat_2d():
    assert_allclose(angle.rot_mat_2d(0.0),
                    np.array([[1., 0.],
                              [0., 1.]]))


def test_angle_mod():
    assert_allclose(angle.angle_mod(-4.0), 2.28318531)
    assert(isinstance(angle.angle_mod(-4.0), float))
    assert_allclose(angle.angle_mod([-4.0]), [2.28318531])
    assert(isinstance(angle.angle_mod([-4.0]), np.ndarray))

    assert_allclose(angle.angle_mod([-150.0, 190.0, 350], degree=True),
                    [-150., -170., -10.])

    assert_allclose(angle.angle_mod(-60.0, zero_2_2pi=True, degree=True),
                    [300.])


if __name__ == '__main__':
    conftest.run_this_test(__file__)
