import numpy as np

import conftest  # Add root path to sys.path
from Localization.probabilistic_motion_model import probabilistic_motion_model as m

START_POSE = np.array([0.0, 0.0, 0.0])
TIME_STEP = 2.0  # [s]
# Noise parameters of the numeric checks below
VELOCITY_ALPHAS = (0.02, 0.02, 0.02, 0.02, 0.01, 0.01)
ODOMETRY_ALPHAS = (0.02, 0.02, 0.02, 0.02)


def peak_density(variances):
    """Product of the peaks of zero-mean normal densities."""
    return np.prod([1.0 / np.sqrt(2.0 * np.pi * v) for v in variances])


def velocity_model_peak(control, alphas):
    speed, turn_rate = control
    a1, a2, a3, a4, a5, a6 = alphas
    return peak_density([a1 * speed ** 2 + a2 * turn_rate ** 2,
                         a3 * speed ** 2 + a4 * turn_rate ** 2,
                         a5 * speed ** 2 + a6 * turn_rate ** 2])


def fraction_of_samples_in_mass_region(density_xy, x_values, y_values,
                                       samples, mass):
    level = m.density_level_for_mass(density_xy, mass)
    ix = np.clip(np.searchsorted(x_values, samples[0]), 0, len(x_values) - 1)
    iy = np.clip(np.searchsorted(y_values, samples[1]), 0, len(y_values) - 1)
    return np.mean(density_xy[ix, iy] >= level)


def test_main_without_animation():
    m.show_animation = False
    m.main()


def test_straight_motion():
    control = (1.0, 0.0)
    end_pose = m.noise_free_velocity_motion(control, START_POSE, TIME_STEP)
    np.testing.assert_allclose(end_pose, [2.0, 0.0, 0.0], atol=1e-12)

    density_at_end = m.velocity_motion_model(
        end_pose, control, START_POSE, VELOCITY_ALPHAS, TIME_STEP)
    np.testing.assert_allclose(
        density_at_end, velocity_model_peak(control, VELOCITY_ALPHAS))

    off_path_pose = np.array([2.0, 0.3, 0.0])
    density_off_path = m.velocity_motion_model(
        off_path_pose, control, START_POSE, VELOCITY_ALPHAS, TIME_STEP)
    assert density_off_path < density_at_end


def test_forward_right_turn():
    # Line 7 of Table 5.1 as printed gives about 5e-38 here, because it
    # recovers a negative speed for a forward right turn. With the signed
    # radius, the noise-free end pose gets the highest density, the same as
    # for the mirrored left turn.
    right_turn = (1.0, -0.35)
    left_turn = (1.0, 0.35)
    peak = velocity_model_peak(right_turn, VELOCITY_ALPHAS)
    for control in (right_turn, left_turn):
        end_pose = m.noise_free_velocity_motion(control, START_POSE, TIME_STEP)
        density = m.velocity_motion_model(
            end_pose, control, START_POSE, VELOCITY_ALPHAS, TIME_STEP)
        np.testing.assert_allclose(density, peak, rtol=1e-6)


def test_backward_motion():
    for control in ((-1.0, 0.35), (-1.0, -0.35)):
        end_pose = m.noise_free_velocity_motion(control, START_POSE, TIME_STEP)
        density = m.velocity_motion_model(
            end_pose, control, START_POSE, VELOCITY_ALPHAS, TIME_STEP)
        np.testing.assert_allclose(
            density, velocity_model_peak(control, VELOCITY_ALPHAS),
            rtol=1e-6)


def test_angle_wrapping_across_pi():
    # Start close to yaw = pi and turn left, so that the end yaw crosses the
    # boundary between pi and -pi. The densities must not depend on where the
    # robot starts, and all angles must stay in [-pi, pi).
    rng = np.random.default_rng(1)
    control = (1.0, 0.35)
    rotated_start = np.array([1.0, -2.0, 3.0])
    end_pose = m.noise_free_velocity_motion(control, rotated_start, TIME_STEP)
    assert end_pose[2] < 0.0  # wrapped from 3.7 rad to 3.7 - 2 pi

    reference_end = m.noise_free_velocity_motion(control, START_POSE, TIME_STEP)
    np.testing.assert_allclose(
        m.velocity_motion_model(end_pose, control, rotated_start,
                                VELOCITY_ALPHAS, TIME_STEP),
        m.velocity_motion_model(reference_end, control, START_POSE,
                                VELOCITY_ALPHAS, TIME_STEP),
        rtol=1e-6)

    odometry = (rotated_start, end_pose)
    reference_odometry = (START_POSE, reference_end)
    np.testing.assert_allclose(
        m.odometry_motion_model(end_pose, odometry, rotated_start,
                                ODOMETRY_ALPHAS),
        m.odometry_motion_model(reference_end, reference_odometry, START_POSE,
                                ODOMETRY_ALPHAS),
        rtol=1e-6)

    for samples in (
            m.sample_velocity_motion_model(control, rotated_start,
                                           VELOCITY_ALPHAS, TIME_STEP,
                                           1000, rng),
            m.sample_odometry_motion_model(odometry, rotated_start,
                                           ODOMETRY_ALPHAS, 1000, rng)):
        assert np.all(samples[2] >= -np.pi) and np.all(samples[2] < np.pi)


def test_odometry_pure_rotation():
    # Without translation, the whole turn goes into the second rotation.
    rotation1, translation, rotation2 = m.odometry_motion_components(
        START_POSE, np.array([0.0, 0.0, 0.5]))
    assert rotation1 == 0.0
    assert translation == 0.0
    np.testing.assert_allclose(rotation2, 0.5)


def test_velocity_samples_agree_with_closed_form():
    rng = np.random.default_rng(0)
    for control in ((1.0, 0.0), (1.0, 0.35), (1.0, -0.35)):
        samples = m.sample_velocity_motion_model(
            control, START_POSE, VELOCITY_ALPHAS, TIME_STEP, 5000, rng)
        x_values, y_values, yaw_values, grid = m.pose_grid_around(samples)
        density_xy = m.velocity_motion_model(
            grid, control, START_POSE, VELOCITY_ALPHAS,
            TIME_STEP).sum(axis=2)
        fraction = fraction_of_samples_in_mass_region(
            density_xy, x_values, y_values, samples, 0.9)
        assert 0.87 < fraction < 0.93


def test_odometry_samples_agree_with_closed_form():
    # Table 5.6 takes the noise variance from the measured odometry, Table 5.5
    # from the candidate pose, so the two distributions are close but not
    # equal: about 93 % of the samples fall in the 90 % region of the closed
    # form, not 90 %.
    rng = np.random.default_rng(0)
    for control in ((1.0, 0.0), (1.0, 0.35), (1.0, -0.35)):
        odometry = (START_POSE,
                    m.noise_free_velocity_motion(control, START_POSE, TIME_STEP))
        samples = m.sample_odometry_motion_model(
            odometry, START_POSE, ODOMETRY_ALPHAS, 5000, rng)
        x_values, y_values, yaw_values, grid = m.pose_grid_around(samples)
        density = m.odometry_motion_model(grid, odometry, START_POSE,
                                          ODOMETRY_ALPHAS)
        translation = m.odometry_motion_components(START_POSE, grid)[1]
        density_xy = (density / translation).sum(axis=2)
        fraction = fraction_of_samples_in_mass_region(
            density_xy, x_values, y_values, samples, 0.9)
        assert 0.87 < fraction < 0.97


def test_odometry_density_over_pose_integrates_to_one():
    # Table 5.5 returns a density over (rotation1, translation, rotation2).
    # Divided by the Jacobian, the translation, it becomes a density over
    # (x, y, yaw) and integrates to about one.
    odometry = (START_POSE, np.array([2.0 * np.cos(0.35), 2.0 * np.sin(0.35),
                                      0.7]))
    x_values = np.linspace(-0.5, 3.0, 101)
    y_values = np.linspace(-1.0, 2.5, 101)
    yaw_values = np.linspace(-np.pi, np.pi, 181, endpoint=False)
    grid = np.stack(np.meshgrid(x_values, y_values, yaw_values, indexing="ij"))
    cell = ((x_values[1] - x_values[0]) * (y_values[1] - y_values[0])
            * (yaw_values[1] - yaw_values[0]))
    density = m.odometry_motion_model(grid, odometry, START_POSE,
                                      ODOMETRY_ALPHAS)
    translation = m.odometry_motion_components(START_POSE, grid)[1]
    integral_noise_space = density.sum() * cell
    integral_pose_space = (density / np.maximum(translation, 1e-9)).sum() * cell
    assert integral_noise_space > 1.5
    assert abs(integral_pose_space - 1.0) < 0.1


if __name__ == '__main__':
    conftest.run_this_test(__file__)
