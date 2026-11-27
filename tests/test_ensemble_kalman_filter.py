import conftest  # Add root path to sys.path
import numpy as np
from Localization.ensemble_kalman_filter import ensemble_kalman_filter as m


def test_calc_covariance_ignores_velocity_component():
    # Two particles with a 4-dimensional state [x, y, yaw, v].
    # calc_covariance must return the 3x3 covariance of the first three
    # state components. The large velocity component (row 3) must not leak
    # into it.
    #
    # This is a regression test for a broadcasting bug: (px[:, i] - xEst)
    # subtracted a (4,) vector from a (4, 1) column, which NumPy broadcast
    # to a (4, 4) array instead of a 3-element residual, so the covariance
    # mixed in the unrelated velocity component.
    px = np.array([[1.0, -1.0],
                   [2.0, -2.0],
                   [3.0, -3.0],
                   [100.0, -100.0]])
    x_est = np.mean(px, axis=1).reshape(4, 1)

    cov = m.calc_covariance(x_est, px)

    residual = px[0:3] - x_est[0:3]
    expected = (residual @ residual.T) / px.shape[1]

    assert cov.shape == (3, 3)
    np.testing.assert_allclose(cov, expected)
    # The velocity component (value 100) must not dominate the position /
    # orientation covariance.
    assert cov[0, 0] < 1e3


def test_calc_covariance_is_symmetric_positive_semidefinite():
    rng = np.random.default_rng(12345)
    x_est = np.zeros((4, 1))
    px = rng.normal(size=(4, m.NP))
    x_est[:, 0] = np.mean(px, axis=1)

    cov = m.calc_covariance(x_est, px)

    np.testing.assert_allclose(cov, cov.T)
    eigenvalues = np.linalg.eigvalsh(cov)
    assert np.all(eigenvalues >= -1e-12)


def test_simulation_runs():
    np.random.seed(12345)
    m.show_animation = False
    original_sim_time = m.SIM_TIME
    try:
        m.SIM_TIME = 3.0  # keep the smoke test fast and deterministic
        m.main()
    finally:
        m.SIM_TIME = original_sim_time


if __name__ == '__main__':
    conftest.run_this_test(__file__)
