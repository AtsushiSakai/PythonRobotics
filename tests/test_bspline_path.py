import conftest
import numpy as np
import pytest
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


if __name__ == '__main__':
    conftest.run_this_test(__file__)
