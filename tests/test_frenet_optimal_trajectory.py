import copy
from unittest.mock import Mock

import conftest
import numpy as np
import pytest

from PathPlanning.FrenetOptimalTrajectory import frenet_optimal_trajectory as m


@pytest.mark.parametrize("lateral", [
    m.HighSpeedLateralMovementStrategy,
    m.LowSpeedLateralMovementStrategy,
])
@pytest.mark.parametrize("longitudinal", [
    m.VelocityKeepingLongitudinalMovementStrategy,
    m.MergingAndStoppingLongitudinalMovementStrategy,
])
def test_scenario(monkeypatch, lateral, longitudinal):
    monkeypatch.setattr(m, "show_animation", False)
    monkeypatch.setattr(m, "SIM_LOOP", 5)
    monkeypatch.setattr(m, "LATERAL_MOVEMENT_STRATEGY", lateral())
    monkeypatch.setattr(m, "LONGITUDINAL_MOVEMENT_STRATEGY", longitudinal())
    m.main()


@pytest.mark.parametrize("strategy", [
    m.VelocityKeepingLongitudinalMovementStrategy(),
    m.MergingAndStoppingLongitudinalMovementStrategy(),
])
def test_longitudinal_samples_match_scalar_evaluation(strategy):
    duration = 4.4
    paths = strategy.calc_longitudinal_trajectory(2.5, 0.2, duration, 1.0)
    if isinstance(strategy, m.VelocityKeepingLongitudinalMovementStrategy):
        targets = np.arange(m.TARGET_SPEED - m.D_T_S * m.N_S_SAMPLE,
                            m.TARGET_SPEED + m.D_T_S * m.N_S_SAMPLE, m.D_T_S)
        polynomials = [m.QuarticPolynomial(1.0, 2.5, 0.2, target, 0.0, duration)
                       for target in targets]
    else:
        targets = np.arange(m.STOP_S - m.D_S * m.N_STOP_S_SAMPLE,
                            m.STOP_S + m.D_S * m.N_STOP_S_SAMPLE, m.D_S)
        polynomials = [m.QuinticPolynomial(1.0, 2.5, 0.2, target, 0.0, 0.0, duration)
                       for target in targets]
    assert len(paths) == len(polynomials)
    for path, polynomial in zip(paths, polynomials):
        for field, evaluate in zip(
            ["s", "s_d", "s_dd", "s_ddd"],
            [polynomial.calc_point, polynomial.calc_first_derivative,
             polynomial.calc_second_derivative, polynomial.calc_third_derivative],
        ):
            assert isinstance(getattr(path, field), list)
            np.testing.assert_allclose(getattr(path, field),
                                       [evaluate(t) for t in path.t], atol=1e-12)


@pytest.mark.parametrize("strategy", [
    m.HighSpeedLateralMovementStrategy(),
    m.LowSpeedLateralMovementStrategy(),
])
def test_lateral_samples_match_scalar_evaluation(strategy):
    duration = 4.4
    path = m.VelocityKeepingLongitudinalMovementStrategy().calc_longitudinal_trajectory(
        2.5, 0.2, duration, 1.0)[0]
    original = copy.deepcopy(vars(path))
    result = strategy.calc_lateral_trajectory(path, -1.0, 0.3, 0.1, -0.02, duration)
    if isinstance(strategy, m.HighSpeedLateralMovementStrategy):
        polynomial = m.QuinticPolynomial(
            0.3, 0.1 * path.s_d[0], -0.02 * path.s_d[0]**2 + 0.1 * path.s_dd[0],
            -1.0, 0.0, 0.0, duration)
        expected = []
        for t, s_d, s_dd in zip(path.t, path.s_d, path.s_dd):
            inverse = 1.0 / (s_d + 1e-6) + 1e-6
            d_d = polynomial.calc_first_derivative(t) * inverse
            d_dd = (polynomial.calc_second_derivative(t) - d_d * s_dd) * inverse**2
            expected.append([polynomial.calc_point(t), d_d, d_dd,
                             polynomial.calc_third_derivative(t)])
    else:
        polynomial = m.QuinticPolynomial(
            0.3, 0.1, -0.02, -1.0, 0.0, 0.0, path.s[-1] - path.s[0])
        expected = [[polynomial.calc_point(s - path.s[0]),
                     polynomial.calc_first_derivative(s - path.s[0]),
                     polynomial.calc_second_derivative(s - path.s[0]),
                     polynomial.calc_third_derivative(s - path.s[0])]
                    for s in path.s]
    for field, values in zip(["d", "d_d", "d_dd", "d_ddd"], np.array(expected).T):
        assert isinstance(getattr(result, field), list)
        np.testing.assert_allclose(getattr(result, field), values, atol=1e-12)
    assert vars(path) == original
    # Candidates must retain independent, mutable longitudinal histories.
    result.s.pop(0)
    assert vars(path) == original


@pytest.mark.parametrize("strategy", [
    m.HighSpeedLateralMovementStrategy(),
    m.LowSpeedLateralMovementStrategy(),
])
def test_global_paths_reuse_reference_geometry(monkeypatch, strategy):
    monkeypatch.setattr(m, "LATERAL_MOVEMENT_STRATEGY", strategy)
    csp = m.cubic_spline_planner.CubicSpline2D(m.WX, m.WY)
    longitudinal = m.VelocityKeepingLongitudinalMovementStrategy().calc_longitudinal_trajectory(
        2.5, 0.2, 4.4, 1.0)[0]
    paths = [strategy.calc_lateral_trajectory(longitudinal, d, 0.3, 0.1, -0.02, 4.4)
             for d in [-0.5, 0.0, 0.5]]
    # Independent scalar conversions provide the reference for every candidate.
    expected = []
    for path in paths:
        expected.append([m.CartesianFrenetConverter.frenet_to_cartesian(
            s, *csp.calc_position(s), csp.calc_yaw(s), csp.calc_curvature(s),
            csp.calc_curvature_rate(s), [s, path.s_d[i], path.s_dd[i]],
            [path.d[i], path.d_d[i], path.d_dd[i]]) for i, s in enumerate(path.s)])
    methods = ["calc_position", "calc_yaw", "calc_curvature", "calc_curvature_rate"]
    for name in methods:
        monkeypatch.setattr(csp, name, Mock(wraps=getattr(csp, name)))
    for run in [1, 2]:
        results = m.calc_global_paths(copy.deepcopy(paths), csp)
        for result, values in zip(results, expected):
            np.testing.assert_allclose(
                np.array([result.x, result.y, result.yaw, result.c, result.v, result.a]).T,
                values, atol=1e-12)
        # Each planning step owns its cache; later steps reevaluate the reference.
        for name in methods:
            assert getattr(csp, name).call_count == run * len(set(longitudinal.s))


def test_global_paths_stop_at_first_out_of_range_sample(monkeypatch):
    csp = m.cubic_spline_planner.CubicSpline2D([0, 10, 20], [0, 0, 0])
    monkeypatch.setattr(csp, "calc_position", Mock(wraps=csp.calc_position))
    path = m.FrenetPath()
    path.s = [1.0, 3.0, 21.0, 5.0]
    path.s_d = [1.0] * 4
    path.s_dd = path.d = path.d_d = path.d_dd = [0.0] * 4
    results = m.calc_global_paths([path, copy.deepcopy(path)], csp)
    assert csp.calc_position.call_count == 3
    for result in results:
        np.testing.assert_allclose(result.x, [1.0, 3.0])
        assert len(result.y) == len(result.yaw) == len(result.c) == 2
    assert m.calc_global_paths([], csp) == []


if __name__ == '__main__':
    conftest.run_this_test(__file__)
