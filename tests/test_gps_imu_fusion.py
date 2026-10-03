import conftest
import numpy as np
import matplotlib.pyplot as plt
import pytest

from Localization.gps_imu_fusion import gps_imu_fusion as m


def test_body_frame_acceleration_and_bias_compensation():
    state = np.array([1.0, 2.0, 3.0, 4.0, np.pi / 2, 0.2, -0.1, 0.03])
    imu = np.array([2.2, -0.1, 0.13])
    original = state.copy()
    result = m.motion_model(state, imu, 0.5)
    # Body x points along world y, so only vy has acceleration.
    expected = [2.5, 4.25, 3.0, 5.0, np.pi / 2 + 0.05, 0.2, -0.1, 0.03]
    np.testing.assert_allclose(result, expected, atol=1e-12)
    np.testing.assert_array_equal(state, original)


def test_motion_jacobians_match_finite_differences():
    state = np.array([1.0, 2.0, -0.5, 1.5, 0.7, 0.2, -0.1, 0.03])
    imu = np.array([0.8, -0.3, 0.2])
    dt, eps = 0.13, 1e-6
    f, g = m.motion_jacobians(state, imu, dt)
    for index in range(8):
        delta = np.eye(8)[index] * eps
        numeric = (m.motion_model(state + delta, imu, dt)
                   - m.motion_model(state - delta, imu, dt)) / (2 * eps)
        np.testing.assert_allclose(f[:, index], numeric, atol=1e-9)
    for index in range(3):
        delta = np.eye(3)[index] * eps
        numeric = (m.motion_model(state, imu + delta, dt)
                   - m.motion_model(state, imu - delta, dt)) / (2 * eps)
        np.testing.assert_allclose(g[:, index], numeric, atol=1e-9)


def test_prediction_noise_units_at_rest():
    state = np.zeros(8)
    _, covariance = m.predict(state, np.zeros((8, 8)), np.zeros(3), dt=0.2)
    np.testing.assert_allclose(covariance[0, 0], (0.5 * 0.2**2 * m.IMU_STD[0])**2)
    np.testing.assert_allclose(covariance[0, 2], 0.5 * 0.2**3 * m.IMU_STD[0]**2)
    np.testing.assert_allclose(covariance[4, 4], (0.2 * m.IMU_STD[2])**2)
    np.testing.assert_allclose(np.diag(covariance)[5:], m.BIAS_RW_STD**2 * 0.2)


def test_gps_update_matches_linear_kalman_result():
    state = np.zeros(8)
    covariance = np.eye(8)
    covariance[0, 2] = covariance[2, 0] = 0.4
    measurement = np.array([2.0, -1.0])
    updated, posterior = m.update_gps(state, covariance, measurement)
    expected = np.zeros(8)
    expected[:2] = measurement / (1.0 + m.GPS_STD**2)
    expected[2] = 0.4 * measurement[0] / (1.0 + m.GPS_STD**2)
    np.testing.assert_allclose(updated, expected)
    expected_covariance = covariance - covariance[:, :2] @ covariance[:2, :] / (
        1.0 + m.GPS_STD**2)
    np.testing.assert_allclose(posterior, expected_covariance, atol=1e-12)
    np.testing.assert_array_equal(state, np.zeros(8))
    assert covariance[0, 0] == 1.0


def test_heading_wraps_during_prediction_and_correction():
    state = np.zeros(8)
    state[4] = np.pi - 0.01
    result, _ = m.predict(state, np.eye(8), np.array([0.0, 0.0, 0.3]), dt=0.1)
    np.testing.assert_allclose(result[4], -np.pi + 0.02)
    covariance = np.eye(8)
    covariance[0, 4] = covariance[4, 0] = 0.5
    result, _ = m.update_gps(state, covariance, np.array([1.0, 0.0]))
    assert -np.pi <= result[4] < 0.0


@pytest.mark.parametrize("seed", [0, 7, 42])
def test_fusion_reduces_drift_with_gps_outage(seed):
    history = m.simulate(seed=seed)
    estimates = history["estimate"]
    fused_error = estimates[:, :2] - history["truth"][:, :2]
    dead_error = history["dead_reckoning"][:, :2] - history["truth"][:, :2]
    fused_rmse = np.sqrt(np.mean(np.sum(fused_error**2, axis=1)))
    dead_rmse = np.sqrt(np.mean(np.sum(dead_error**2, axis=1)))
    assert fused_rmse < 2.0
    assert fused_rmse < dead_rmse / 5.0
    assert np.all(np.isfinite(estimates))
    assert np.all(np.abs(estimates[:, 4]) <= np.pi)
    covariance = history["covariance"]
    np.testing.assert_allclose(covariance, covariance.transpose(0, 2, 1), atol=1e-12)
    assert np.linalg.eigvalsh(covariance).min() >= -1e-12


def test_gps_schedule_outage_and_recovery():
    history = m.simulate()
    steps = np.arange(len(history["time"]))
    available = np.isfinite(history["gps"]).all(axis=1)
    expected = (steps > 0) & (steps % m.GPS_INTERVAL == 0)
    expected &= (history["time"] < 20.0) | (history["time"] >= 30.0)
    np.testing.assert_array_equal(available, expected)
    uncertainty = np.trace(history["covariance"][:, :2, :2], axis1=1, axis2=2)
    assert uncertainty[599] > uncertainty[399]  # grows without GPS
    assert uncertainty[600] < uncertainty[599]  # shrinks on GPS recovery
    continuous = m.simulate(gps_outage=None)
    # Removing GPS corrections must not change simulated IMU noise or truth.
    np.testing.assert_array_equal(history["dead_reckoning"], continuous["dead_reckoning"])
    np.testing.assert_array_equal(history["truth"], continuous["truth"])
    np.testing.assert_array_equal(history["gps"][available], continuous["gps"][available])


def test_main_without_animation(monkeypatch):
    monkeypatch.setattr(m, "show_animation", False)
    monkeypatch.setattr(m, "SIM_TIME", 2.0)
    history = m.main()
    assert len(history["time"]) == 41


def test_animation_renders(tmp_path):
    history = m.simulate(duration=1.0, gps_outage=(0.4, 0.8))
    animation = m.create_animation(history)
    try:
        animation.save(tmp_path / "fusion.gif", writer="pillow", fps=5)
    finally:
        plt.close(plt.gcf())


if __name__ == '__main__':
    conftest.run_this_test(__file__)
