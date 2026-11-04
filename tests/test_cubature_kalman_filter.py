import conftest
import numpy as np
import pytest
from Localization.cubature_kalman_filter import cubature_kalman_filter as m


def test1():
    m.show_final = False
    m.show_animation = False
    m.show_ellipse = False
    m.main()


@pytest.mark.parametrize("yaw_rate", [-0.4, 0.08, 1.0])
@pytest.mark.parametrize("root_coupling", [-0.01, 0.0, 0.01])
def test_prediction_yaw_moments_match_one_time_step(yaw_rate, root_coupling):
    state = np.array([[1.5], [-2.0], [0.4], [3.0], [yaw_rate]])
    root = np.diag([0.2, 0.3, 0.1, 0.4, 0.02])
    root[4, 2] = root_coupling
    covariance = root @ root.T
    state_before = state.copy()
    covariance_before = covariance.copy()
    process_noise_before = m.q.copy()

    predicted_state, predicted_covariance = m.cubature_prediction(state, covariance)

    # Yaw is affine even though the position transition is nonlinear.
    expected_yaw = state[2, 0] + m.dt * state[4, 0]
    expected_yaw_variance = (
        covariance[2, 2]
        + 2 * m.dt * covariance[2, 4]
        + m.dt**2 * covariance[4, 4]
        + m.q[2, 2]
    )
    expected_yaw_rate_covariance = (
        covariance[2, 4] + m.dt * covariance[4, 4] + m.q[2, 4]
    )
    np.testing.assert_allclose(predicted_state[2, 0], expected_yaw, atol=1e-14)
    np.testing.assert_allclose(
        predicted_covariance[2, 2], expected_yaw_variance, atol=1e-14
    )
    np.testing.assert_allclose(
        predicted_covariance[2, 4], expected_yaw_rate_covariance, atol=1e-14
    )
    np.testing.assert_array_equal(state, state_before)
    np.testing.assert_array_equal(covariance, covariance_before)
    np.testing.assert_array_equal(m.q, process_noise_before)


@pytest.mark.parametrize("yaw_rate", [-0.4, 0.15])
def test_prediction_matches_independent_ctrv_cubature_moments(yaw_rate):
    state = np.array([[1.5], [-2.0], [0.4], [3.0], [yaw_rate]])
    covariance = np.diag([0.04, 0.09, 0.01, 0.16, 0.0004])
    dimension = state.shape[0]
    offsets = np.sqrt(dimension) * np.diag(np.sqrt(np.diag(covariance)))
    points = np.hstack((state + offsets, state - offsets))

    # Integrate the circular arc using its midpoint heading and sinc length,
    # independently of the production divided sine/cosine differences.
    half_turn = points[4] * m.dt / 2
    distance = points[3] * m.dt * np.sinc(half_turn / np.pi)
    propagated = points.copy()
    propagated[0] += distance * np.cos(points[2] + half_turn)
    propagated[1] += distance * np.sin(points[2] + half_turn)
    propagated[2] += points[4] * m.dt
    expected_state = np.mean(propagated, axis=1, keepdims=True)
    centered = propagated - expected_state
    expected_covariance = centered @ centered.T / (2 * dimension) + m.q

    predicted_state, predicted_covariance = m.cubature_prediction(state, covariance)

    np.testing.assert_allclose(predicted_state, expected_state, rtol=1e-12, atol=1e-14)
    np.testing.assert_allclose(
        predicted_covariance, expected_covariance, rtol=1e-12, atol=1e-14
    )
    np.testing.assert_allclose(predicted_covariance, predicted_covariance.T, atol=1e-14)
    assert np.linalg.eigvalsh(predicted_covariance).min() >= 0


if __name__ == '__main__':
    conftest.run_this_test(__file__)
