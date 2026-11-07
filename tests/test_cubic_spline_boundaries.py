import math
import unittest

from PathPlanning.CubicSpline.cubic_spline_planner import CubicSpline1D, CubicSpline2D


class TestSplineBoundaries(unittest.TestCase):
    def test_final_knot_uses_last_polynomial(self):
        spline = CubicSpline1D([0., 1.], [1., 3.])
        assert [method(1.) for method in (spline.calc_position, spline.calc_first_derivative,
                                          spline.calc_second_derivative, spline.calc_third_derivative)] == [3., 2., 0., 0.]


    def test_repeated_knots(self):
        with self.assertRaises(ValueError):
            CubicSpline1D([0, 1, 1], [0, 1, 2])


    def test_minimum_knots(self):
        with self.assertRaises(ValueError):
            CubicSpline1D([0], [1])


    def test_unpaired_samples(self):
        with self.assertRaises(ValueError):
            CubicSpline1D([0, 1], [1, 2, 3])


    def test_queries_beyond_path(self):
        spline = CubicSpline2D([0, 1], [0, 1])
        for s in (-1, 3):
            assert spline.calc_yaw(s) is None
            assert spline.calc_curvature(s) is None
            assert spline.calc_curvature_rate(s) is None
        assert math.isclose(spline.calc_yaw(0), math.pi / 4)
