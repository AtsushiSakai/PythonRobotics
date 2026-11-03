import conftest
import math

import pytest
from scipy.interpolate import CubicSpline

from PathPlanning.CubicSpline.cubic_spline_planner import (
    CubicSpline1D,
    CubicSpline2D,
)


@pytest.mark.parametrize("x, y", [
    ([0.0, 2.0], [1.0, 5.0]),
    ([0.0, 1.0, 2.0], [0.0, 1.0, 0.0]),
])
@pytest.mark.parametrize("method, order", [
    ("calc_position", 0),
    ("calc_first_derivative", 1),
    ("calc_second_derivative", 2),
    ("calc_third_derivative", 3),
])
def test_1d_final_knot(x, y, method, order):
    spline = CubicSpline1D(x, y)
    reference = CubicSpline(x, y, bc_type="natural")
    evaluate = getattr(spline, method)

    for value in [x[0], 0.5, 1.0, 1.5, x[-1]]:
        assert evaluate(value) == pytest.approx(reference(value, order))

    assert evaluate(x[0] - 0.1) is None
    assert evaluate(x[-1] + 0.1) is None


def test_2d_final_knot():
    spline = CubicSpline2D([0.0, 2.0, 4.0], [0.0, 3.0, 6.0])
    endpoint = spline.s[-1]

    assert spline.calc_position(endpoint) == pytest.approx((4.0, 6.0))
    assert spline.calc_yaw(endpoint) == pytest.approx(math.atan2(3.0, 2.0))
    assert spline.calc_curvature(endpoint) == pytest.approx(0.0)
    assert spline.calc_curvature_rate(endpoint) == pytest.approx(0.0)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
