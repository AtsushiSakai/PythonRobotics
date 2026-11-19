import conftest
import numpy as np
import pytest
from SLAM.EKFSLAM import ekf_slam as m


def test_1():
    m.show_animation = False
    m.main()


@pytest.mark.parametrize("landmark_count", [0, 1, 2])
@pytest.mark.parametrize("yaw", [0.0, 0.7, -1.2])
@pytest.mark.parametrize("velocity", [0.0, 2.0])
def test_prediction_covariance_matches_motion_linearization(
        landmark_count, yaw, velocity):
    state_size = m.STATE_SIZE + m.LM_SIZE * landmark_count
    state = np.arange(state_size, dtype=float).reshape(-1, 1)
    state[2, 0] = yaw
    control = np.array([[velocity], [0.3]])
    # Include pose/map correlations, not just a diagonal pose covariance.
    factor = np.arange(1, state_size ** 2 + 1, dtype=float).reshape(
        state_size, state_size) / state_size
    covariance = factor @ factor.T + np.eye(state_size)

    def predict_state(value):
        predicted = value.copy()
        predicted[:m.STATE_SIZE] = m.motion_model(
            value[:m.STATE_SIZE], control)
        return predicted

    # Differentiate the actual state transition independently of jacob_motion.
    step = 1e-5
    jacobian = np.empty((state_size, state_size))
    for column in range(state_size):
        perturbation = np.zeros_like(state)
        perturbation[column, 0] = step
        jacobian[:, column] = (
            (predict_state(state + perturbation)
             - predict_state(state - perturbation)) / (2.0 * step)
        ).ravel()
    expected_covariance = jacobian @ covariance @ jacobian.T
    expected_covariance[:m.STATE_SIZE, :m.STATE_SIZE] += m.Cx

    predicted_state, predicted_covariance = m.ekf_slam(
        state.copy(), covariance.copy(), control, np.empty((0, 3)))

    np.testing.assert_allclose(predicted_state, predict_state(state))
    np.testing.assert_allclose(predicted_covariance, expected_covariance,
                               rtol=1e-9, atol=1e-9)
    np.testing.assert_allclose(predicted_covariance[3:, 3:], covariance[3:, 3:])


if __name__ == '__main__':
    conftest.run_this_test(__file__)
