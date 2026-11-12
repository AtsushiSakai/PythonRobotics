import numpy as np
import conftest
from SLAM.ICPMatching import icp_matching as m


def test_1():
    m.show_animation = False
    m.main()


def test_2():
    m.show_animation = False
    m.main_3d_points()


def test_residual_uses_nearest_neighbors():
    previous = np.array([[0., 3., 8.], [0., 0., 0.]])
    for current, expected, residual in [
            (previous[:, [2, 0, 1]], [2, 0, 1], 0.),
            (np.array([[8., 1.], [0., 0.]]), [2, 0], 1.)]:
        indexes, error = m.nearest_neighbor_association(previous, current)
        np.testing.assert_array_equal(indexes, expected)
        assert error == residual


if __name__ == '__main__':
    conftest.run_this_test(__file__)
