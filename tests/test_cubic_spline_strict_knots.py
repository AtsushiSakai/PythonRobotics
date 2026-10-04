"""Reject zero-width spline intervals without altering valid spline mathematics."""
from PathPlanning.CubicSpline import cubic_spline_planner as module
import warnings

import numpy as np
import pytest
from scipy.interpolate import CubicSpline


@pytest.mark.parametrize("x", [[0, 0, 1], [0, 1, 1, 2], [0, 1, 2, 2], [0, 0]])
def test_duplicate_knots_raise_before_numerical_work(x):
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        with pytest.raises(ValueError, match="strictly increasing"):
            module.CubicSpline1D(x, list(range(len(x))))


@pytest.mark.parametrize("x,y", [([0, 1, 1, 2], [0, 2, 2, 3]), ([0, 0], [1, 1])])
def test_consecutive_duplicate_waypoints_raise(x, y):
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        with pytest.raises(ValueError, match="strictly increasing"):
            module.CubicSpline2D(x, y)


def test_decreasing_knots_remain_rejected():
    with pytest.raises(ValueError, match="x coordinates"):
        module.CubicSpline1D([0, 2, 1], [1, 2, 3])


@pytest.mark.parametrize("x,y", [([0, 2], [1, 4]), ([0, 0.5, 2, 4], [1, -2, 3, 0])])
@pytest.mark.parametrize("order", range(4))
def test_valid_spline_matches_scipy(x, y, order):
    spline = module.CubicSpline1D(x, y)
    reference = CubicSpline(x, y, bc_type="natural")
    # The independent final-knot defect is already covered by upstream PR #1433.
    # This change only rejects repeated knots and deliberately leaves that code alone.
    query = np.linspace(x[0], x[-1], 41, endpoint=False)
    method = (spline.calc_position, spline.calc_first_derivative,
              spline.calc_second_derivative, spline.calc_third_derivative)[order]
    np.testing.assert_allclose([method(v) for v in query], reference(query, nu=order),
                               rtol=1e-10, atol=1e-10)

