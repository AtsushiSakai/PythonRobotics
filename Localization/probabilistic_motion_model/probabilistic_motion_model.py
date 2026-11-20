"""
Probabilistic motion models from Probabilistic Robotics, chapter 5

This example implements the two motion models of Thrun, Burgard and Fox,
"Probabilistic Robotics" (MIT Press, 2005), chapter 5:

- the velocity motion model with control u = (v, omega) and noise
  parameters alpha_1 ... alpha_6 (Tables 5.1 and 5.3), and
- the odometry motion model with control u = (odometry pose at t-1,
  odometry pose at t) and noise parameters alpha_1 ... alpha_4
  (Tables 5.5 and 5.6).

Each model has a closed-form function that evaluates the density
p(x_t | u_t, x_{t-1}) at a hypothesised pose and a sampler that draws a pose
from the same distribution. The samplers are the prediction step of Monte
Carlo localization; the densities are the same noise model written as a
function of the pose, as used in grid-based Bayes filters.

States are (x, y, theta). Distances are in meters, angles in radians and
time in seconds. Angles are kept in [-pi, pi).

The inverse models have two identifiability limits. The velocity model
recovers the heading change only modulo 2*pi, so it assumes that both the
turning angle and the total heading change of one step are smaller than pi.
The odometry decomposition needs the direction of the translation; a
rotation in place is handled by putting the whole rotation in the second
rotation.

Only NumPy and Matplotlib are required.

"""
import math
import random

import matplotlib.pyplot as plt
import numpy as np

# Default noise parameters of the velocity motion model, alpha_1 ... alpha_6.
ALPHA_VELOCITY = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

# Default noise parameters of the odometry motion model, alpha_1 ... alpha_4.
ALPHA_ODOMETRY = np.array([0.1, 0.1, 0.1, 0.1])

DT = 1.0  # time step [s]
N_SAMPLE = 500  # number of samples drawn per motion
STRAIGHT_LINE_EPS = 1e-12  # quantity below this is treated as zero

show_animation = True


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi).

    Parameters
    ----------
    angle : float
        Angle [rad].

    Returns
    -------
    float
        Equivalent angle in [-pi, pi).

    """
    normalized = math.atan2(math.sin(angle), math.cos(angle))
    if normalized == math.pi:
        return -math.pi
    return normalized


def prob_normal_distribution(a, variance):
    """Density of the zero-centered normal distribution with the variance.

    This is the function prob(a, b) of Table 5.2 with the normal distribution.
    A zero variance is treated as a point mass: the density is 1 for a == 0
    and 0 elsewhere. The book's distribution is degenerate in that case.

    Parameters
    ----------
    a : float
        Value whose density is computed.
    variance : float
        Variance b of the distribution, b >= 0.

    Returns
    -------
    float
        Probability density.

    """
    if variance <= 0.0:
        return 1.0 if abs(a) < STRAIGHT_LINE_EPS else 0.0

    return math.exp(-a * a / (2.0 * variance)) / math.sqrt(
        2.0 * math.pi * variance)


def sample_normal_distribution(variance):
    """Draw a sample from the zero-centered normal distribution.

    Table 5.4 implements the function sample(b) with a sum of twelve uniform
    samples; this function uses a Gaussian random variable instead.

    Parameters
    ----------
    variance : float
        Variance b of the distribution, b >= 0.

    Returns
    -------
    float
        Random sample with zero mean and the given variance.

    """
    if variance <= 0.0:
        return 0.0

    return random.gauss(0.0, math.sqrt(variance))


def _velocity_motion_parameters(x_prev, x, dt, signed_radius=True):
    """Recover the noise-free controls (v_hat, omega_hat, gamma_hat).

    This is Table 5.1, lines 2-9: the inverse motion model that maps the two
    poses back to the controls that would carry the robot between them.

    The radius of the circular segment is unsigned in Table 5.1, line 5.
    When v and omega have opposite signs, the recovered speed then gets the
    wrong sign, because the unsigned radius does not encode the direction of
    travel. With signed_radius=True the signed radius of equation 5.17 is
    used instead, which is the projection of the centre of the circle onto
    the left normal of the initial heading.

    Two poses determine the heading change only modulo 2*pi, so the
    difference of the two atan2 values is wrapped to (-pi, pi]. This assumes
    that both the turning angle |omega*dt| and the total heading change
    |omega*dt + gamma*dt| are smaller than pi, which is the small-time-step
    regime of the book. When the two positions coincide (for example a
    rotation in place) the centre of the circle is undefined; the recovered
    omega is then zero and the whole heading change is assigned to gamma.

    Parameters
    ----------
    x_prev : array_like, shape (3,)
        Previous pose (x, y, theta) [m, m, rad].
    x : array_like, shape (3,)
        Hypothesised pose (x', y', theta') [m, m, rad].
    dt : float
        Time between the two poses [s].
    signed_radius : bool, optional
        Use the signed radius of equation 5.17 (True, default) or the
        unsigned radius of Table 5.1 line 5 (False).

    Returns
    -------
    tuple of float
        Recovered (v_hat, omega_hat, gamma_hat).

    """
    x0, y0, theta0 = x_prev
    x1, y1, theta1 = x

    # Lines 2-4: center (x_star, y_star) of the circle.
    numerator = (x0 - x1) * math.cos(theta0) + (y0 - y1) * math.sin(theta0)
    denominator = (y0 - y1) * math.cos(theta0) - (x0 - x1) * math.sin(theta0)

    if abs(denominator) < STRAIGHT_LINE_EPS:
        # The center of the circle is undefined: either the robot moved on a
        # straight line (omega = 0), or the two positions coincide (for
        # example a rotation in place). The projection of the displacement
        # on the heading keeps the direction of travel, so a backward motion
        # gives a negative speed. With coincident positions the split of the
        # heading change between omega and gamma is not identifiable, and
        # the whole change is assigned to gamma.
        v_hat = ((x1 - x0) * math.cos(theta0)
                 + (y1 - y0) * math.sin(theta0)) / dt
        omega_hat = 0.0
    else:
        mu = 0.5 * numerator / denominator
        x_star = 0.5 * (x0 + x1) + mu * (y0 - y1)
        y_star = 0.5 * (y0 + y1) + mu * (x1 - x0)

        # Lines 5-6: radius and heading change. The difference is wrapped:
        # without that, the same motion seen from a different global heading
        # can be recovered as omega - 2*pi.
        radius = math.hypot(x0 - x_star, y0 - y_star)
        delta_theta = normalize_angle(
            math.atan2(y1 - y_star, x1 - x_star)
            - math.atan2(y0 - y_star, x0 - x_star))

        if signed_radius:
            # Signed radius of equation 5.17: the center lies at
            # (x0, y0) + r * (-sin(theta0), cos(theta0)).
            radius = (-(x_star - x0) * math.sin(theta0)
                      + (y_star - y0) * math.cos(theta0))

        # Lines 7-8: recovered translational and rotational velocities.
        v_hat = delta_theta / dt * radius
        omega_hat = delta_theta / dt

    # Line 9: recovered final rotation. The commanded one is zero.
    gamma_hat = normalize_angle(theta1 - theta0) / dt - omega_hat

    return v_hat, omega_hat, gamma_hat


def velocity_motion_model(u, x_prev, x, alpha, dt, signed_radius=True):
    """Density p(x | u, x_prev) of the velocity motion model.

    This is Table 5.1: the commanded control and the hypothesised poses are
    used to recover the noise-free controls (v_hat, omega_hat, gamma_hat),
    then the density of the differences under the motion noise is returned.

    The radius of the circular segment is unsigned in Table 5.1, line 5,
    which gives the recovered speed the wrong sign when v and omega have
    opposite signs. See ``_velocity_motion_parameters``. With
    signed_radius=False the printed algorithm is reproduced, for comparison.

    Parameters
    ----------
    u : array_like, shape (2,)
        Commanded velocities (v, omega) [m/s, rad/s].
    x_prev : array_like, shape (3,)
        Previous pose (x, y, theta) [m, m, rad].
    x : array_like, shape (3,)
        Hypothesised pose (x', y', theta') [m, m, rad].
    alpha : array_like, shape (6,)
        Noise parameters alpha_1 ... alpha_6.
        The variances are alpha_1 v^2 + alpha_2 omega^2,
        alpha_3 v^2 + alpha_4 omega^2 and alpha_5 v^2 + alpha_6 omega^2.
    dt : float
        Time between the two poses [s].
    signed_radius : bool, optional
        Use the signed radius of equation 5.17 (True, default) or the
        unsigned radius of Table 5.1 line 5 (False).

    Returns
    -------
    float
        Probability density p(x | u, x_prev).

    """
    v, omega = u
    v_hat, omega_hat, gamma_hat = _velocity_motion_parameters(
        x_prev, x, dt, signed_radius)

    # Line 10: densities of the individual errors.
    var_v = alpha[0] * v * v + alpha[1] * omega * omega
    var_omega = alpha[2] * v * v + alpha[3] * omega * omega
    var_gamma = alpha[4] * v * v + alpha[5] * omega * omega

    return (prob_normal_distribution(v - v_hat, var_v)
            * prob_normal_distribution(omega - omega_hat, var_omega)
            * prob_normal_distribution(gamma_hat, var_gamma))


def sample_velocity_motion_model(u, x_prev, alpha, dt):
    """Sample a pose from the velocity motion model.

    This is Table 5.3. The commanded velocities are perturbed with the motion
    noise, then the pose is obtained by integrating the noisy velocities.

    Parameters
    ----------
    u : array_like, shape (2,)
        Commanded velocities (v, omega) [m/s, rad/s].
    x_prev : array_like, shape (3,)
        Previous pose (x, y, theta) [m, m, rad].
    alpha : array_like, shape (6,)
        Noise parameters alpha_1 ... alpha_6.
    dt : float
        Duration of the motion [s].

    Returns
    -------
    numpy.ndarray, shape (3,)
        Sampled pose (x', y', theta').

    """
    v, omega = u
    x, y, theta = x_prev

    # Lines 2-4: perturb the commanded control.
    var_v = alpha[0] * v * v + alpha[1] * omega * omega
    var_omega = alpha[2] * v * v + alpha[3] * omega * omega
    var_gamma = alpha[4] * v * v + alpha[5] * omega * omega
    v_hat = v + sample_normal_distribution(var_v)
    omega_hat = omega + sample_normal_distribution(var_omega)
    gamma_hat = sample_normal_distribution(var_gamma)

    # Lines 5-6: integrate the noisy velocities on the circle.
    # The half-angle form is the same expression as Table 5.3, rearranged to
    # avoid the cancellation of sin(theta + omega*dt) - sin(theta).
    if abs(omega_hat) < STRAIGHT_LINE_EPS:
        x_new = x + v_hat * dt * math.cos(theta)
        y_new = y + v_hat * dt * math.sin(theta)
    else:
        half = 0.5 * omega_hat * dt
        x_new = x + 2.0 * (v_hat / omega_hat) * math.cos(
            theta + half) * math.sin(half)
        y_new = y + 2.0 * (v_hat / omega_hat) * math.sin(
            theta + half) * math.sin(half)

    # Line 7: final orientation, including the extra rotation gamma_hat*dt.
    theta_new = normalize_angle(theta + omega_hat * dt + gamma_hat * dt)

    return np.array([x_new, y_new, theta_new])


def _odometry_motion_parameters(x_prev, x):
    """Split the relative motion of two poses into three parameters.

    This is the decomposition of equations (5.34)-(5.36) and lines 2-4 of
    Tables 5.5 and 5.6: a rotation, a translation and a second rotation.

    The direction of the translation is undefined when the two positions
    coincide (for example a robot rotating in place). In that case the whole
    rotation is put in delta_rot2 and delta_rot1 is zero, so the
    decomposition does not depend on the global heading of the frame.

    Parameters
    ----------
    x_prev : array_like, shape (3,)
        Pose at the beginning of the motion.
    x : array_like, shape (3,)
        Pose at the end of the motion.

    Returns
    -------
    tuple of float
        (delta_rot1, delta_trans, delta_rot2).

    """
    x0, y0, theta0 = x_prev
    x1, y1, theta1 = x

    delta_trans = math.hypot(x1 - x0, y1 - y0)
    if delta_trans < STRAIGHT_LINE_EPS:
        # No translation direction: keep the whole rotation in delta_rot2.
        # Otherwise atan2(0, 0) = 0 would make delta_rot1 = -theta0 and the
        # noise would depend on the global heading of the odometry frame.
        delta_rot1 = 0.0
        delta_rot2 = normalize_angle(theta1 - theta0)
    else:
        delta_rot1 = normalize_angle(
            math.atan2(y1 - y0, x1 - x0) - theta0)
        delta_rot2 = normalize_angle(theta1 - theta0 - delta_rot1)

    return delta_rot1, delta_trans, delta_rot2


def odometry_motion_model(u, x_prev, x, alpha):
    """Density p(x | u, x_prev) of the odometry motion model.

    This is Table 5.5. The control u is the pair of poses reported by the
    odometry, u = (odometry pose at t-1, odometry pose at t). The odometry
    and the hypothesised poses are both decomposed into a rotation, a
    translation and a second rotation; the density of their differences is
    returned.

    The variances are those of equations (5.44)-(5.46), which use the
    odometry values, so the noise model is fixed by the control and matches
    the sampler of Table 5.6.

    Parameters
    ----------
    u : tuple of array_like
        Odometry control (x_bar_prev, x_bar), each of shape (3,).
    x_prev : array_like, shape (3,)
        Previous pose (x, y, theta).
    x : array_like, shape (3,)
        Hypothesised pose (x', y', theta').
    alpha : array_like, shape (4,)
        Noise parameters alpha_1 ... alpha_4.

    Returns
    -------
    float
        Probability density p(x | u, x_prev).

    """
    u_prev, u_curr = u
    delta_rot1, delta_trans, delta_rot2 = _odometry_motion_parameters(
        u_prev, u_curr)
    delta_rot1_hat, delta_trans_hat, delta_rot2_hat = (
        _odometry_motion_parameters(x_prev, x))

    # Lines 8-10 of Table 5.5 with the variances of (5.44)-(5.46).
    var_rot1 = (alpha[0] * delta_rot1 * delta_rot1
                + alpha[1] * delta_trans * delta_trans)
    var_trans = (alpha[2] * delta_trans * delta_trans
                 + alpha[3] * delta_rot1 * delta_rot1
                 + alpha[3] * delta_rot2 * delta_rot2)
    var_rot2 = (alpha[0] * delta_rot2 * delta_rot2
                + alpha[1] * delta_trans * delta_trans)

    p1 = prob_normal_distribution(
        normalize_angle(delta_rot1 - delta_rot1_hat), var_rot1)
    p2 = prob_normal_distribution(delta_trans - delta_trans_hat, var_trans)
    p3 = prob_normal_distribution(
        normalize_angle(delta_rot2 - delta_rot2_hat), var_rot2)

    return p1 * p2 * p3


def sample_odometry_motion_model(u, x_prev, alpha):
    """Sample a pose from the odometry motion model.

    This is Table 5.6. The odometry is decomposed into a rotation, a
    translation and a second rotation; each is perturbed with its motion
    noise and the pose is reconstructed from the perturbed values.

    Parameters
    ----------
    u : tuple of array_like
        Odometry control (x_bar_prev, x_bar), each of shape (3,).
    x_prev : array_like, shape (3,)
        Previous pose (x, y, theta).
    alpha : array_like, shape (4,)
        Noise parameters alpha_1 ... alpha_4.

    Returns
    -------
    numpy.ndarray, shape (3,)
        Sampled pose (x', y', theta').

    """
    u_prev, u_curr = u
    delta_rot1, delta_trans, delta_rot2 = _odometry_motion_parameters(
        u_prev, u_curr)

    # Lines 5-7: perturb the three motion parameters.
    var_rot1 = (alpha[0] * delta_rot1 * delta_rot1
                + alpha[1] * delta_trans * delta_trans)
    var_trans = (alpha[2] * delta_trans * delta_trans
                 + alpha[3] * delta_rot1 * delta_rot1
                 + alpha[3] * delta_rot2 * delta_rot2)
    var_rot2 = (alpha[0] * delta_rot2 * delta_rot2
                + alpha[1] * delta_trans * delta_trans)

    delta_rot1_hat = delta_rot1 - sample_normal_distribution(var_rot1)
    delta_trans_hat = delta_trans - sample_normal_distribution(var_trans)
    delta_rot2_hat = delta_rot2 - sample_normal_distribution(var_rot2)

    # Lines 8-10: reconstruct the pose.
    x, y, theta = x_prev
    x_new = x + delta_trans_hat * math.cos(theta + delta_rot1_hat)
    y_new = y + delta_trans_hat * math.sin(theta + delta_rot1_hat)
    theta_new = normalize_angle(theta + delta_rot1_hat + delta_rot2_hat)

    return np.array([x_new, y_new, theta_new])


def calc_noise_free_pose(u, x_prev, dt):
    """Pose after the commanded velocity motion without noise.

    Parameters
    ----------
    u : array_like, shape (2,)
        Commanded velocities (v, omega).
    x_prev : array_like, shape (3,)
        Previous pose.
    dt : float
        Duration of the motion [s].

    Returns
    -------
    numpy.ndarray, shape (3,)
        Noise-free pose after the motion.

    """
    zero_alpha = np.zeros(6)
    return sample_velocity_motion_model(u, x_prev, zero_alpha, dt)


def calc_density_grid(u, x_prev, alpha, dt, x_range, y_range, theta_range):
    """Marginal density over (x, y) on a grid, for plotting.

    The three-dimensional density p(x, y, theta | u, x_prev) is summed over
    a regular grid of headings, which is the projection shown in Figures 5.3
    and 5.4 of the book.

    Parameters
    ----------
    u : array_like, shape (2,)
        Commanded velocities (v, omega).
    x_prev : array_like, shape (3,)
        Previous pose.
    alpha : array_like, shape (6,)
        Noise parameters alpha_1 ... alpha_6.
    dt : float
        Duration of the motion [s].
    x_range, y_range : array_like
        Grid coordinates along x and y [m].
    theta_range : array_like
        Grid coordinates of the heading [rad].

    Returns
    -------
    numpy.ndarray, shape (len(x_range), len(y_range))
        Sum over the headings of the density times the heading step.

    """
    density = np.zeros((len(x_range), len(y_range)))
    delta_theta = theta_range[1] - theta_range[0]

    for i, x in enumerate(x_range):
        for j, y in enumerate(y_range):
            for theta in theta_range:
                density[i, j] += velocity_motion_model(
                    u, x_prev, (x, y, theta), alpha, dt) * delta_theta

    return density


def plot_velocity_samples(u, x_prev, alpha, dt):
    """Animate samples and density contour of the velocity motion model.

    Parameters
    ----------
    u : array_like, shape (2,)
        Commanded velocities (v, omega).
    x_prev : array_like, shape (3,)
        Previous pose.
    alpha : array_like, shape (6,)
        Noise parameters alpha_1 ... alpha_6.
    dt : float
        Duration of the motion [s].

    """
    x_nominal = calc_noise_free_pose(u, x_prev, dt)
    samples = np.array([sample_velocity_motion_model(u, x_prev, alpha, dt)
                        for _ in range(N_SAMPLE)])

    x_range = np.linspace(x_nominal[0] - 2.0, x_nominal[0] + 2.0, 41)
    y_range = np.linspace(x_nominal[1] - 2.0, x_nominal[1] + 2.0, 41)
    theta_range = np.linspace(x_nominal[2] - math.pi,
                              x_nominal[2] + math.pi, 25)
    density = calc_density_grid(u, x_prev, alpha, dt, x_range, y_range,
                                theta_range)

    step = max(1, N_SAMPLE // 25)
    for i in range(step, N_SAMPLE + step, step):
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        levels = np.linspace(0.1, 0.9, 5) * density.max()
        if levels[-1] > 0.0:
            plt.contour(x_range, y_range, density.T, levels=levels,
                        colors='k', linewidths=0.8)
        plt.plot(samples[:i, 0], samples[:i, 1], '.r', markersize=3)
        plt.plot(x_prev[0], x_prev[1], 'sk', markersize=8)
        plt.plot(x_nominal[0], x_nominal[1], '*b', markersize=12)
        plt.arrow(x_prev[0], x_prev[1],
                  0.5 * math.cos(x_prev[2]), 0.5 * math.sin(x_prev[2]),
                  head_width=0.1, color='k')
        plt.axis("equal")
        plt.grid(True)
        plt.title(f"v={u[0]:.2f} m/s, omega={u[1]:.2f} rad/s, "
                  f"{i} samples")
        plt.pause(0.001)


def main():
    """Animate the velocity motion model for three motion commands."""
    print(__file__ + " start!!")

    motions = [
        ("straight", 1.0, 0.0),
        ("left turn", 1.0, 0.4),
        ("right turn", 1.0, -0.4),
    ]
    x_prev = np.zeros(3)

    for name, v, omega in motions:
        print(f"velocity motion model: {name}")
        if show_animation:
            plot_velocity_samples((v, omega), x_prev, ALPHA_VELOCITY, DT)


if __name__ == '__main__':
    main()
