import math
import random

import numpy as np
import pytest

import conftest
from Localization.probabilistic_motion_model import probabilistic_motion_model as m

ALPHA_VELOCITY = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
ALPHA_ODOMETRY = np.array([0.1, 0.1, 0.1, 0.1])
DT = 1.0


def max_density(variances):
    """Peak of a product of zero-centered normal densities."""
    return float(np.prod([1.0 / math.sqrt(2.0 * math.pi * b)
                          for b in variances]))


def velocity_volume(u, alpha, dt, x_prev, span=2.5, n_xy=41, n_theta=41):
    """Integrate the velocity density over the pose space on a grid."""
    x_nominal = m.calc_noise_free_pose(u, x_prev, dt)
    xs = np.linspace(x_nominal[0] - span, x_nominal[0] + span, n_xy)
    ys = np.linspace(x_nominal[1] - span, x_nominal[1] + span, n_xy)
    thetas = np.linspace(x_nominal[2] - math.pi, x_nominal[2] + math.pi,
                         n_theta)
    volume = 0.0
    for x in xs:
        for y in ys:
            for theta in thetas:
                volume += m.velocity_motion_model(
                    u, x_prev, (x, y, theta), alpha, dt)
    return volume * (xs[1] - xs[0]) * (ys[1] - ys[0]) * (thetas[1] - thetas[0])


def odometry_poses(v, omega, dt):
    """Odometry control that reports the noise-free velocity motion."""
    u_prev = np.zeros(3)
    u_curr = m.calc_noise_free_pose((v, omega), u_prev, dt)
    return u_prev, u_curr


def test_straight_line_motion():
    random.seed(0)
    x_prev = np.zeros(3)
    alpha_zero = np.zeros(6)

    x = m.sample_velocity_motion_model((1.0, 0.0), x_prev, alpha_zero, DT)
    np.testing.assert_allclose(x, [1.0, 0.0, 0.0], atol=1e-12)

    # A backward straight motion keeps the sign of the speed.
    x_back = m.sample_velocity_motion_model((-1.0, 0.0), x_prev, alpha_zero, DT)
    np.testing.assert_allclose(x_back, [-1.0, 0.0, 0.0], atol=1e-12)

    # At the noise-free pose the errors are zero, so the density is the peak
    # of the product of the three normal densities.
    expected = max_density([ALPHA_VELOCITY[0] * 1.0,
                            ALPHA_VELOCITY[2] * 1.0,
                            ALPHA_VELOCITY[4] * 1.0])
    for u, pose in (((1.0, 0.0), x), ((-1.0, 0.0), x_back)):
        density = m.velocity_motion_model(u, x_prev, pose, ALPHA_VELOCITY,
                                          DT)
        assert density == pytest.approx(expected, rel=1e-9)


def test_velocity_angle_wrapping():
    random.seed(0)
    x_prev = np.array([0.0, 0.0, 3.0])
    u = (0.5, 0.5)

    x = m.sample_velocity_motion_model(u, x_prev, np.zeros(6), DT)
    # 3.0 + 0.5 rad crosses pi, so the heading is wrapped to [-pi, pi).
    assert -math.pi <= x[2] < math.pi
    assert x[2] == math.atan2(math.sin(3.5), math.cos(3.5))

    expected = max_density([
        ALPHA_VELOCITY[0] * 0.5 + ALPHA_VELOCITY[1] * 0.5,
        ALPHA_VELOCITY[2] * 0.5 + ALPHA_VELOCITY[3] * 0.5,
        ALPHA_VELOCITY[4] * 0.5 + ALPHA_VELOCITY[5] * 0.5])
    density = m.velocity_motion_model(u, x_prev, x, ALPHA_VELOCITY, DT)
    assert density == pytest.approx(expected, rel=1e-9)


def test_forward_right_turn():
    random.seed(0)
    u = (1.0, -0.35)
    dt = 2.0
    x_prev = np.zeros(3)
    x_true = m.sample_velocity_motion_model(u, x_prev, np.zeros(6), dt)
    np.testing.assert_allclose(x_true, [1.840622, -0.671879, -0.7], atol=1e-6)

    # The true pose is the mode of the density when the signed radius is used.
    expected = max_density([
        ALPHA_VELOCITY[0] * 1.0 + ALPHA_VELOCITY[1] * 0.35,
        ALPHA_VELOCITY[2] * 1.0 + ALPHA_VELOCITY[3] * 0.35,
        ALPHA_VELOCITY[4] * 1.0 + ALPHA_VELOCITY[5] * 0.35])
    density = m.velocity_motion_model(u, x_prev, x_true, ALPHA_VELOCITY, dt)
    assert density == pytest.approx(expected, rel=1e-9)

    # Table 5.1 line 5 with the unsigned radius recovers v_hat = -1 instead
    # of +1, so the true pose gets an almost vanishing density.
    density_printed = m.velocity_motion_model(
        u, x_prev, x_true, ALPHA_VELOCITY, dt, signed_radius=False)
    assert density_printed < 1e-5


def test_velocity_signed_radius_all_turn_directions():
    random.seed(0)
    dt = 2.0
    x_prev = np.zeros(3)
    for v, omega in ((1.0, 0.35), (1.0, -0.35), (-1.0, 0.35), (-1.0, -0.35)):
        u = (v, omega)
        x_true = m.sample_velocity_motion_model(u, x_prev, np.zeros(6), dt)
        v_hat, omega_hat, gamma_hat = m._velocity_motion_parameters(
            x_prev, x_true, dt)
        assert abs(v_hat - v) < 1e-9
        assert abs(omega_hat - omega) < 1e-9
        assert abs(gamma_hat) < 1e-9

        density = m.velocity_motion_model(u, x_prev, x_true, ALPHA_VELOCITY,
                                          dt)
        expected = max_density([
            ALPHA_VELOCITY[0] * abs(v) + ALPHA_VELOCITY[1] * abs(omega),
            ALPHA_VELOCITY[2] * abs(v) + ALPHA_VELOCITY[3] * abs(omega),
            ALPHA_VELOCITY[4] * abs(v) + ALPHA_VELOCITY[5] * abs(omega)])
        assert density == pytest.approx(expected, rel=1e-9)


def test_velocity_frame_invariance():
    # The two atan2 values must be wrapped. Without the wrap, the same
    # motion seen from a different global heading can be recovered as
    # omega - 2*pi, and the density at the true pose becomes zero.
    random.seed(0)
    for omega in (0.4, -0.4, 1.0, 2.0):
        u = (1.0, omega)
        expected = max_density([
            ALPHA_VELOCITY[0] * 1.0 + ALPHA_VELOCITY[1] * abs(omega),
            ALPHA_VELOCITY[2] * 1.0 + ALPHA_VELOCITY[3] * abs(omega),
            ALPHA_VELOCITY[4] * 1.0 + ALPHA_VELOCITY[5] * abs(omega)])
        for theta0 in np.linspace(-math.pi, math.pi, 37)[:-1]:
            x_prev = np.array([0.0, 0.0, float(theta0)])
            x_true = m.sample_velocity_motion_model(
                u, x_prev, np.zeros(6), DT)
            _, omega_hat, _ = m._velocity_motion_parameters(
                x_prev, x_true, DT)
            assert omega_hat == pytest.approx(omega, abs=1e-9)
            density = m.velocity_motion_model(
                u, x_prev, x_true, ALPHA_VELOCITY, DT)
            assert density == pytest.approx(expected, rel=1e-9)


def test_velocity_zero_displacement():
    # A rotation in place leaves the position unchanged, so the book's
    # inverse is 0/0. The convention is omega_hat = 0 with the whole heading
    # change in gamma_hat.
    x_prev = np.zeros(3)
    x = np.array([0.0, 0.0, 0.5])
    v_hat, omega_hat, gamma_hat = m._velocity_motion_parameters(
        x_prev, x, DT)
    assert v_hat == 0.0
    assert omega_hat == 0.0
    assert gamma_hat == pytest.approx(0.5)

    # A stationary command has zero variance: a point mass at the same pose.
    assert m.prob_normal_distribution(0.0, 0.0) == 1.0
    assert m.prob_normal_distribution(0.1, 0.0) == 0.0
    assert m.velocity_motion_model(
        (0.0, 0.0), x_prev, x_prev, ALPHA_VELOCITY, DT) == 1.0
    assert m.velocity_motion_model(
        (0.0, 0.0), x_prev, x, ALPHA_VELOCITY, DT) == 0.0


def test_velocity_large_turn_alias():
    # Two poses determine the heading change only modulo 2*pi, so a turn of
    # more than pi per step is recovered as omega*DT - 2*pi.
    omega = 3.5
    x_prev = np.zeros(3)
    x = m.sample_velocity_motion_model((1.0, omega), x_prev, np.zeros(6), DT)
    _, omega_hat, _ = m._velocity_motion_parameters(x_prev, x, DT)
    assert omega_hat == pytest.approx(m.normalize_angle(omega * DT) / DT)
    assert omega_hat == pytest.approx(omega - 2.0 * math.pi)


def test_velocity_sampler_matches_density():
    random.seed(1)
    u = (1.0, -0.35)
    n_samples = 8000
    x_prev = np.zeros(3)

    errors = np.zeros((n_samples, 3))
    for i in range(n_samples):
        x = m.sample_velocity_motion_model(u, x_prev, ALPHA_VELOCITY, DT)
        v_hat, omega_hat, gamma_hat = m._velocity_motion_parameters(
            x_prev, x, DT)
        errors[i] = (v_hat - u[0], omega_hat - u[1], gamma_hat)

    # The density is a zero-centered normal in these same errors, so the
    # empirical variances must be the variances it uses.
    variances = [
        ALPHA_VELOCITY[0] * abs(u[0]) + ALPHA_VELOCITY[1] * abs(u[1]),
        ALPHA_VELOCITY[2] * abs(u[0]) + ALPHA_VELOCITY[3] * abs(u[1]),
        ALPHA_VELOCITY[4] * abs(u[0]) + ALPHA_VELOCITY[5] * abs(u[1]),
    ]
    np.testing.assert_allclose(np.var(errors, axis=0), variances, rtol=0.1,
                               atol=0.0)


def test_velocity_density_volume():
    # The density of Table 5.1 is a density over the noise variables, not
    # over the pose. Integrating it over the pose space therefore gives
    # approximately the determinant of the map from (v, omega, gamma) to
    # (x, y, theta).
    random.seed(0)
    x_prev = np.zeros(3)
    u = (1.0, -0.35)
    dt = 2.0

    volume = velocity_volume(u, ALPHA_VELOCITY, dt, x_prev)

    # Determinant of the forward map, by finite differences.
    h = 1e-6

    def pose(v, omega, gamma):
        return m.sample_velocity_motion_model(
            (v, omega), x_prev, np.zeros(6), dt) + np.array(
                [0.0, 0.0, gamma * dt])

    jacobian = np.zeros((3, 3))
    base = pose(u[0], u[1], 0.0)
    for k, delta in enumerate(((h, 0.0, 0.0), (0.0, h, 0.0), (0.0, 0.0, h))):
        jacobian[:, k] = (pose(u[0] + delta[0], u[1] + delta[1], delta[2])
                          - base) / h

    determinant = abs(np.linalg.det(jacobian))
    assert volume > 5.0  # not 1: the Jacobian is missing from Table 5.1
    np.testing.assert_allclose(volume, determinant, rtol=0.1)


def test_odometry_angle_wrapping():
    # The motion crosses pi: the heading goes from 3.0 to 3.5 rad, which is
    # stored as 3.5 - 2*pi. All angular differences must be wrapped back to
    # [-pi, pi), otherwise the second rotation error diverges.
    x_prev = np.array([0.0, 0.0, 3.0])
    theta_end = 3.5 - 2.0 * math.pi
    u_prev = np.array([0.0, 0.0, 3.0])
    u_curr = np.array([math.cos(3.5), math.sin(3.5), theta_end])

    delta_rot1, delta_trans, delta_rot2 = m._odometry_motion_parameters(
        u_prev, u_curr)
    assert delta_rot1 == pytest.approx(0.5)
    assert delta_trans == pytest.approx(1.0)
    assert delta_rot2 == pytest.approx(0.0, abs=1e-12)

    x = m.sample_odometry_motion_model((u_prev, u_curr), x_prev, np.zeros(4))
    np.testing.assert_allclose(x, [math.cos(3.5), math.sin(3.5), theta_end],
                               atol=1e-12)

    # The end pose, stored with the wrapped heading, is the density peak.
    density = m.odometry_motion_model((u_prev, u_curr), x_prev, x,
                                      ALPHA_ODOMETRY)
    variances = [
        ALPHA_ODOMETRY[0] * abs(delta_rot1)
        + ALPHA_ODOMETRY[1] * abs(delta_trans),
        ALPHA_ODOMETRY[2] * abs(delta_trans)
        + ALPHA_ODOMETRY[3] * abs(delta_rot1 + delta_rot2),
        ALPHA_ODOMETRY[0] * abs(delta_rot2)
        + ALPHA_ODOMETRY[1] * abs(delta_trans),
    ]
    assert density == pytest.approx(max_density(variances), rel=1e-9)


def test_odometry_zero_translation():
    # A pure rotation has no translation direction. The decomposition must
    # not depend on the global heading of the odometry frame.
    rotation = 0.1
    x_prev = np.zeros(3)
    x = np.array([0.0, 0.0, rotation])
    # The variances are 0, alpha_4*rotation and alpha_1*rotation, so the
    # density is the product of two normal densities and a point mass.
    var_trans = ALPHA_ODOMETRY[3] * abs(rotation)
    var_rot2 = ALPHA_ODOMETRY[0] * abs(rotation)
    expected = (1.0 / math.sqrt(2.0 * math.pi * var_trans)
                * 1.0 / math.sqrt(2.0 * math.pi * var_rot2))
    for theta0 in (0.0, 1.0, -2.0):
        u_prev = np.array([0.0, 0.0, theta0])
        u_curr = np.array([0.0, 0.0, theta0 + rotation])
        assert m._odometry_motion_parameters(u_prev, u_curr) == pytest.approx(
            (0.0, 0.0, rotation), abs=1e-12)
        density = m.odometry_motion_model(
            (u_prev, u_curr), x_prev, x, ALPHA_ODOMETRY)
        assert density == pytest.approx(expected)

    # The same relative motion and the same seed give the same sample.
    random.seed(3)
    sample_a = m.sample_odometry_motion_model(
        (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, rotation])),
        x_prev, ALPHA_ODOMETRY)
    random.seed(3)
    sample_b = m.sample_odometry_motion_model(
        (np.array([0.0, 0.0, 1.0]),
         np.array([0.0, 0.0, 1.0 + rotation])),
        x_prev, ALPHA_ODOMETRY)
    np.testing.assert_allclose(sample_a, sample_b)


def test_odometry_negative_translation_is_two_to_one():
    # The forward map is two-to-one: (rot1, trans, rot2) and
    # (rot1 + pi, -trans, rot2 + pi) give the same pose. The inverse of
    # Table 5.5 returns the positive-translation branch, which is what the
    # density uses.
    rot1, trans, rot2 = 0.2, 0.5, 0.3
    x_prev = np.zeros(3)
    x_forward = np.array([trans * math.cos(rot1),
                          trans * math.sin(rot1),
                          m.normalize_angle(rot1 + rot2)])
    x_backward = np.array([-trans * math.cos(rot1 + math.pi),
                           -trans * math.sin(rot1 + math.pi),
                           m.normalize_angle(rot1 + rot2 + 2.0 * math.pi)])
    np.testing.assert_allclose(x_forward, x_backward, atol=1e-12)

    # The inverse returns the positive-translation branch of the same pose.
    recovered_rot1, recovered_trans, recovered_rot2 = (
        m._odometry_motion_parameters(x_prev, x_backward))
    assert recovered_rot1 == pytest.approx(rot1)
    assert recovered_trans == pytest.approx(trans)
    assert recovered_rot2 == pytest.approx(rot2)


def test_odometry_density_peak():
    random.seed(0)
    x_prev = np.zeros(3)
    u = odometry_poses(1.0, 0.4, DT)
    x_true = m.sample_odometry_motion_model(u, x_prev, np.zeros(4))

    delta_rot1, delta_trans, delta_rot2 = m._odometry_motion_parameters(*u)
    variances = [
        ALPHA_ODOMETRY[0] * abs(delta_rot1)
        + ALPHA_ODOMETRY[1] * abs(delta_trans),
        ALPHA_ODOMETRY[2] * abs(delta_trans)
        + ALPHA_ODOMETRY[3] * abs(delta_rot1 + delta_rot2),
        ALPHA_ODOMETRY[0] * abs(delta_rot2)
        + ALPHA_ODOMETRY[1] * abs(delta_trans),
    ]
    density = m.odometry_motion_model(u, x_prev, x_true, ALPHA_ODOMETRY)
    assert density == pytest.approx(max_density(variances), rel=1e-9)

    # The density falls away when the hypothesised pose is wrong.
    density_near = m.odometry_motion_model(
        u, x_prev, x_true + np.array([0.2, 0.0, 0.0]), ALPHA_ODOMETRY)
    density_far = m.odometry_motion_model(
        u, x_prev, x_true + np.array([2.0, 0.0, 0.0]), ALPHA_ODOMETRY)
    assert density_far < density_near < density
    assert density_far < density * 1e-3


def test_odometry_sampler_matches_density():
    # delta_trans is much larger than its standard deviation here. Otherwise
    # the sampled translation can become negative, and the decomposition of
    # equations (5.34)-(5.36) then reports a rotation by pi instead.
    random.seed(2)
    u = odometry_poses(1.0, -0.35, 2.0)
    n_samples = 8000
    x_prev = np.zeros(3)

    delta_rot1, delta_trans, delta_rot2 = m._odometry_motion_parameters(*u)
    errors = np.zeros((n_samples, 3))
    for i in range(n_samples):
        x = m.sample_odometry_motion_model(u, x_prev, ALPHA_ODOMETRY)
        rot1_hat, trans_hat, rot2_hat = m._odometry_motion_parameters(
            x_prev, x)
        errors[i] = (delta_rot1 - rot1_hat,
                     delta_trans - trans_hat,
                     m.normalize_angle(delta_rot2 - rot2_hat))

    variances = [
        ALPHA_ODOMETRY[0] * abs(delta_rot1)
        + ALPHA_ODOMETRY[1] * abs(delta_trans),
        ALPHA_ODOMETRY[2] * abs(delta_trans)
        + ALPHA_ODOMETRY[3] * abs(delta_rot1 + delta_rot2),
        ALPHA_ODOMETRY[0] * abs(delta_rot2)
        + ALPHA_ODOMETRY[1] * abs(delta_trans),
    ]
    np.testing.assert_allclose(np.var(errors, axis=0), variances, rtol=0.1,
                               atol=0.0)


def test_odometry_density_volume():
    # The density of Table 5.5 is also a density over the noise variables.
    # Its integral over the pose space is approximately delta_trans, the
    # Jacobian of the map from (delta_rot1, delta_trans, delta_rot2) to
    # (x, y, theta). Dividing by delta_trans gives a density that integrates
    # to one.
    random.seed(0)
    x_prev = np.zeros(3)
    dt = 2.0
    u = odometry_poses(1.0, -0.35, dt)
    x_nominal = m.sample_odometry_motion_model(u, x_prev, np.zeros(4))
    delta_trans = math.hypot(x_nominal[0] - x_prev[0],
                             x_nominal[1] - x_prev[1])

    xs = np.linspace(x_nominal[0] - 2.5, x_nominal[0] + 2.5, 41)
    ys = np.linspace(x_nominal[1] - 2.5, x_nominal[1] + 2.5, 41)
    thetas = np.linspace(x_nominal[2] - math.pi, x_nominal[2] + math.pi, 41)
    volume = 0.0
    for x in xs:
        for y in ys:
            for theta in thetas:
                volume += m.odometry_motion_model(
                    u, x_prev, (x, y, theta), ALPHA_ODOMETRY)
    volume *= (xs[1] - xs[0]) * (ys[1] - ys[0]) * (thetas[1] - thetas[0])

    np.testing.assert_allclose(volume, delta_trans, rtol=0.05)
    np.testing.assert_allclose(volume / delta_trans, 1.0, rtol=0.05)


def test_main():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
