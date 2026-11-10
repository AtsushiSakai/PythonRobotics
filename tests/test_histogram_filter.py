import conftest
import numpy as np
import pytest
from Localization.histogram_filter import histogram_filter as m


def test1():
    m.show_animation = False
    m.SIM_TIME = 1.0
    m.main()


@pytest.mark.parametrize("readonly", [False, True])
def test_observation_preserves_clean_control(readonly):
    control = m.calc_control_input()
    original = control.copy()
    control.flags.writeable = not readonly
    state = np.zeros((4, 1))
    landmarks = np.array([[100.0, 100.0]])

    next_state, ranges, measured_control = m.observation(state, control, landmarks)

    np.testing.assert_array_equal(control, original)
    np.testing.assert_array_equal(next_state, m.motion_model(state, original))
    assert ranges.shape == (0, 3)
    assert not np.shares_memory(measured_control, control)
    measured_control[:] = 0.0
    np.testing.assert_array_equal(control, original)


def test_reused_control_keeps_true_trajectory_independent_of_sensor_noise():
    control = m.calc_control_input()
    original = control.copy()
    state = np.zeros((4, 1))
    expected = state.copy()
    landmarks = np.array([[100.0, 100.0]])

    for _ in range(3):
        state, _, _ = m.observation(state, control, landmarks)
        expected = m.motion_model(expected, original)
    np.testing.assert_array_equal(control, original)
    np.testing.assert_allclose(state, expected)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
