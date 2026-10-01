import conftest  # Add root path to sys.path
import matplotlib.pyplot as plt
from numpy.testing import assert_allclose
import numpy as np

from utils import angle, plot


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


def test_plot_circle_uses_data_coordinates():
    figure, axes = plt.subplots()

    circle = plot.plot_circle(2.0, -1.0, 0.75, ax=axes)

    expected_bounds = [1.25, -1.75, 2.75, -0.25]
    assert circle in axes.patches
    assert axes.get_aspect() == 1.0
    assert axes.get_adjustable() == "box"

    figure.canvas.draw()
    bounds = circle.get_window_extent()
    assert_allclose(bounds.width, bounds.height)
    assert_allclose(
        bounds.transformed(axes.transData.inverted()).extents,
        expected_bounds,
    )

    figure.set_size_inches(12.0, 3.0)
    figure.canvas.draw()
    resized_bounds = circle.get_window_extent()
    assert_allclose(resized_bounds.width, resized_bounds.height)
    assert_allclose(
        resized_bounds.transformed(axes.transData.inverted()).extents,
        expected_bounds,
    )

    plt.close(figure)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
