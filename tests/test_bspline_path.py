import conftest
import numpy as np
import pytest
from scipy.interpolate import make_interp_spline
from PathPlanning.BSplinePath import bspline_path


def test_list_input():
    way_point_x = [-1.0, 3.0, 4.0, 2.0, 1.0]
    way_point_y = [0.0, -3.0, 1.0, 1.0, 3.0]
    n_course_point = 50  # sampling number

    rax, ray, heading, curvature = bspline_path.approximate_b_spline_path(
        way_point_x, way_point_y, n_course_point, s=0.5)

    assert len(rax) == len(ray) == len(heading) == len(curvature)

    rix, riy, heading, curvature = bspline_path.interpolate_b_spline_path(
        way_point_x, way_point_y, n_course_point)

    assert len(rix) == len(riy) == len(heading) == len(curvature)


def test_array_input():
    way_point_x = np.array([-1.0, 3.0, 4.0, 2.0, 1.0])
    way_point_y = np.array([0.0, -3.0, 1.0, 1.0, 3.0])
    n_course_point = 50  # sampling number

    rax, ray, heading, curvature = bspline_path.approximate_b_spline_path(
        way_point_x, way_point_y, n_course_point, s=0.5)

    assert len(rax) == len(ray) == len(heading) == len(curvature)

    rix, riy, heading, curvature = bspline_path.interpolate_b_spline_path(
        way_point_x, way_point_y, n_course_point)

    assert len(rix) == len(riy) == len(heading) == len(curvature)


def test_degree_change():
    way_point_x = np.array([-1.0, 3.0, 4.0, 2.0, 1.0])
    way_point_y = np.array([0.0, -3.0, 1.0, 1.0, 3.0])
    n_course_point = 50  # sampling number

    rax, ray, heading, curvature = bspline_path.approximate_b_spline_path(
        way_point_x, way_point_y, n_course_point, s=0.5, degree=4)

    assert len(rax) == len(ray) == len(heading) == len(curvature)

    rix, riy, heading, curvature = bspline_path.interpolate_b_spline_path(
        way_point_x, way_point_y, n_course_point, degree=4)

    assert len(rix) == len(riy) == len(heading) == len(curvature)

    rax, ray, heading, curvature = bspline_path.approximate_b_spline_path(
        way_point_x, way_point_y, n_course_point, s=0.5, degree=2)

    assert len(rax) == len(ray) == len(heading) == len(curvature)

    rix, riy, heading, curvature = bspline_path.interpolate_b_spline_path(
        way_point_x, way_point_y, n_course_point, degree=2)

    assert len(rix) == len(riy) == len(heading) == len(curvature)

    with pytest.raises(ValueError):
        bspline_path.approximate_b_spline_path(
            way_point_x, way_point_y, n_course_point, s=0.5, degree=1)

    with pytest.raises(ValueError):
        bspline_path.interpolate_b_spline_path(
            way_point_x, way_point_y, n_course_point, degree=1)


@pytest.mark.parametrize("parameter_scale", [0.5, 1.0, 2.0])
@pytest.mark.parametrize("turn_direction", [-1.0, 1.0])
def test_parabola_curvature_is_independent_of_parameter_speed(
        parameter_scale, turn_direction):
    # These quadratic splines describe x=u, y=+/-u**2 exactly.
    knots = np.array([0.0, 1.0, 2.0])
    spline_x = make_interp_spline(knots / parameter_scale, knots, k=2)
    spline_y = make_interp_spline(
        knots / parameter_scale, turn_direction * knots**2, k=2)
    positions = np.array([0.0, 0.5, 1.0, 2.0])

    x, y, heading, curvature = bspline_path._evaluate_spline(
        positions / parameter_scale, spline_x, spline_y)

    np.testing.assert_allclose(x, positions, atol=1e-14)
    np.testing.assert_allclose(y, turn_direction * positions**2, atol=1e-14)
    np.testing.assert_allclose(
        heading, np.arctan2(2.0 * turn_direction * positions, 1.0), atol=1e-14)
    # Closed-form curvature of the parabola, in inverse coordinate units.
    expected = turn_direction * 2.0 / (1.0 + 4.0 * positions**2)**1.5
    np.testing.assert_allclose(curvature, expected, rtol=1e-12, atol=1e-14)


@pytest.mark.parametrize("degree", [2, 3, 4, 5])
@pytest.mark.parametrize("smoothing", [0.0, 0.4])
@pytest.mark.parametrize("coordinate_scale", [0.25, 2.0])
def test_curvature_scales_inversely_with_path_size(
        degree, smoothing, coordinate_scale):
    points = np.array([
        [0.0, 0.0], [0.5, 0.8], [1.0, 1.2], [2.0, -0.5],
        [3.0, 0.7], [3.8, 1.6], [5.0, 0.3], [5.5, 2.2],
    ])

    def evaluate(points, smoothing):
        if smoothing == 0.0:
            return bspline_path.interpolate_b_spline_path(
                points[:, 0], points[:, 1], 31, degree=degree)
        return bspline_path.approximate_b_spline_path(
            points[:, 0], points[:, 1], 31, degree=degree, s=smoothing)

    x, y, heading, curvature = evaluate(points, smoothing)
    scaled_x, scaled_y, scaled_heading, scaled_curvature = evaluate(
        points * coordinate_scale, smoothing * coordinate_scale**2)

    np.testing.assert_allclose(scaled_x, coordinate_scale * x, atol=1e-12)
    np.testing.assert_allclose(scaled_y, coordinate_scale * y, atol=1e-12)
    np.testing.assert_allclose(scaled_heading, heading, atol=1e-12)
    assert np.max(np.abs(curvature)) > 0.1
    np.testing.assert_allclose(
        scaled_curvature, curvature / coordinate_scale, rtol=1e-10, atol=1e-12)


@pytest.mark.parametrize("degree", [2, 3, 4, 5])
def test_straight_path_has_zero_curvature(degree):
    positions = np.linspace(0.0, 2.0, 7)
    x, y, heading, curvature = bspline_path.interpolate_b_spline_path(
        2.0 * positions, -positions, 31, degree=degree)

    np.testing.assert_allclose(y, -x / 2.0, atol=1e-12)
    np.testing.assert_allclose(heading, np.arctan2(-1.0, 2.0), atol=1e-12)
    np.testing.assert_allclose(curvature, 0.0, atol=1e-12)


@pytest.mark.parametrize("coordinate_scale", [1e-110, 1e110])
def test_straight_path_curvature_at_finite_scales(coordinate_scale):
    x, y, heading, curvature = bspline_path.interpolate_b_spline_path(
        coordinate_scale * np.arange(7.0), np.zeros(7), 31, degree=3)

    assert np.isfinite(x).all()
    np.testing.assert_array_equal(y, 0.0)
    np.testing.assert_array_equal(heading, 0.0)
    np.testing.assert_array_equal(curvature, 0.0)


@pytest.mark.parametrize("coordinate_scale", [1e-110, 1e110])
@pytest.mark.parametrize("turn_direction", [-1.0, 1.0])
def test_parabola_curvature_at_finite_scales(coordinate_scale, turn_direction):
    knots = np.array([0.0, 1.0, 2.0])
    spline_x = make_interp_spline(knots, coordinate_scale * knots, k=2)
    spline_y = make_interp_spline(
        knots, coordinate_scale * turn_direction * knots**2, k=2)
    positions = np.array([0.0, 0.5, 1.0, 2.0])

    _, _, _, curvature = bspline_path._evaluate_spline(
        positions, spline_x, spline_y)

    expected = turn_direction * 2.0 / (1.0 + 4.0 * positions**2)**1.5
    np.testing.assert_allclose(
        curvature * coordinate_scale, expected, rtol=1e-12, atol=1e-14)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
