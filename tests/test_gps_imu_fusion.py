import numpy as np
import pytest

from Localization.gps_imu_fusion import gps_imu_fusion as m


def test_straight_line_prediction_and_input_are_finite():
    filter_ = m.GPSIMUEKF(dt=0.1)
    filter_.state[2] = 1.0
    initial_state = filter_.state.copy()

    state = filter_.predict([0.0, 0.0], 0.0)

    assert np.allclose(state[0:2], [0.1, 0.0])
    assert state[m.YAW_INDEX] == pytest.approx(0.0)
    assert np.all(np.isfinite(filter_.covariance))
    assert np.allclose(initial_state[5:], filter_.state[5:])


def test_wrap_angle_handles_periodic_boundary():
    assert m.wrap_angle(np.pi) == pytest.approx(-np.pi)
    assert m.wrap_angle(-np.pi) == pytest.approx(-np.pi)
    assert m.wrap_angle(3.0 * np.pi) == pytest.approx(-np.pi)


def test_jacobian_matches_finite_difference():
    state = np.array([1.0, -2.0, 0.7, -0.4, 1.2, 0.03, -0.02, 0.01])
    acceleration = np.array([0.4, -0.15])
    dt = 0.03
    analytic = m._transition_jacobian(state, acceleration, dt)
    numerical = np.zeros_like(analytic)
    epsilon = 1e-6
    for index in range(m.STATE_SIZE):
        plus = state.copy()
        minus = state.copy()
        plus[index] += epsilon
        minus[index] -= epsilon
        plus_value = m._transition(plus, acceleration, 0.2, dt)
        minus_value = m._transition(minus, acceleration, 0.2, dt)
        difference = plus_value - minus_value
        difference[m.YAW_INDEX] = m.wrap_angle(
            plus_value[m.YAW_INDEX] - minus_value[m.YAW_INDEX]
        )
        numerical[:, index] = difference / (2.0 * epsilon)

    assert np.allclose(analytic, numerical, atol=2e-6)


def test_gps_outlier_gate_preserves_state():
    filter_ = m.GPSIMUEKF(dt=0.1)
    before = filter_.state.copy()

    accepted = filter_.update_gps([100.0, -100.0], gate_threshold=9.21)

    assert not accepted
    assert np.allclose(filter_.state, before)
    assert filter_.last_nis > 9.21


def test_joseph_update_keeps_covariance_symmetric():
    filter_ = m.GPSIMUEKF(dt=0.05)
    filter_.predict([0.2, -0.1], 0.3)
    assert filter_.update_gps([0.1, -0.2])

    assert np.allclose(filter_.covariance, filter_.covariance.T, atol=1e-12)
    assert np.min(np.linalg.eigvalsh(filter_.covariance)) > -1e-10
    assert filter_.state[m.YAW_INDEX] < np.pi
    assert filter_.state[m.YAW_INDEX] >= -np.pi


def test_simulation_fuses_gps_and_imu_deterministically():
    first = m.run_simulation(duration=8.0, dt=0.02, seed=11)
    second = m.run_simulation(duration=8.0, dt=0.02, seed=11)
    fused_error = np.linalg.norm(
        first["estimate"][:, 0:2] - first["truth"][:, 0:2], axis=1
    )
    dead_reckoning_error = np.linalg.norm(
        first["dead_reckoning"][:, 0:2] - first["truth"][:, 0:2], axis=1
    )

    assert np.array_equal(first["estimate"], second["estimate"])
    assert np.mean(fused_error[-100:]) < np.mean(dead_reckoning_error[-100:])
