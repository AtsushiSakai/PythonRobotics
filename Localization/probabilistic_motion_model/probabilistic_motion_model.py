"""
Probabilistic motion models: velocity and odometry

This example implements the two motion models of chapter 5 of the book
Probabilistic Robotics. Each model has two forms:

- a closed form p(x_t | u_t, x_{t-1}): how likely a candidate pose x_t is,
  given the control u_t and the previous pose x_{t-1},
- a sampler: draw poses x_t from that distribution, which is the prediction
  step of a particle filter (Monte Carlo localization).

Velocity model: the control is u = (v, w), a forward speed [m/s] and a turn
rate [rad/s], held for a time step dt [s]. Noise parameters alpha1 to alpha6.
Tables 5.1 (density) and 5.3 (sampler) of the book.

Odometry model: the control is the pair of poses that the wheel odometry
reports at t-1 and at t. Noise parameters alpha1 to alpha4.
Tables 5.5 (density) and 5.6 (sampler) of the book.

A pose is (x [m], y [m], yaw [rad]). The closed-form functions accept a single
pose of shape (3,) or a whole grid of candidate poses of shape (3, ...).

Two details of the book are handled here and explained in the documentation:

1. Line 7 of Table 5.1 computes the speed from the unsigned radius. When v and
   w have opposite signs (for example a forward right turn), the recovered
   speed has the wrong sign. This code uses the signed radius of the circle,
   so both turn directions work.
2. The closed-form functions return a density over the noise variables of the
   model, as in the book, not a density over the pose (x, y, yaw). The
   Jacobian between the two spaces is not included.

author: Alejandro Lorite Mora (@Lorite)

Reference:
    S. Thrun, W. Burgard and D. Fox, Probabilistic Robotics, MIT Press, 2005,
    chapter 5 (Robot Motion).
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import matplotlib.pyplot as plt
import numpy as np

from utils.angle import angle_mod

# The example parameters make the heading noise larger than the speed noise,
# which shows the typical banana shape of the sample cloud.

# Noise parameters of the velocity model (alpha1 to alpha6 of the book).
# alpha1, alpha2: speed error from speed and from turn rate
# alpha3, alpha4: turn rate error from speed and from turn rate
# alpha5, alpha6: final rotation error from speed and from turn rate
VELOCITY_MODEL_ALPHAS = (0.01, 0.01, 0.1, 0.1, 0.01, 0.01)

# Noise parameters of the odometry model (alpha1 to alpha4 of the book).
# alpha1: rotation error from rotation, alpha2: rotation error from translation
# alpha3: translation error from translation,
# alpha4: translation error from rotation
ODOMETRY_MODEL_ALPHAS = (0.01, 0.05, 0.005, 0.005)

TIME_STEP = 2.0  # duration of one control [s]
NUMBER_OF_SAMPLES = 1000  # samples drawn per motion
MIN_TRANSLATION = 0.01  # [m] below this, the direction of travel is undefined
MIN_LATERAL_OFFSET = 1e-9  # [m] below this, a motion is treated as straight
MIN_VARIANCE = 1e-12  # lower bound of a variance, avoids a division by zero
GRID_RESOLUTION = 81  # grid cells per axis in x and y for the density plot
YAW_RESOLUTION = 121  # grid cells over yaw for the density plot
PROBABILITY_MASS_LEVEL = 0.9  # mass inside the plotted density contour

show_animation = True


def normalize_angle(angle):
    """
    Wrap angles into [-pi, pi) and keep the shape of the input.

    Parameters
    ----------
    angle : float or numpy.ndarray
        Angle or array of angles [rad].

    Returns
    -------
    numpy.ndarray
        Wrapped angles [rad], same shape as the input.
    """
    angle = np.asarray(angle, dtype=float)
    return angle_mod(angle).reshape(angle.shape)


def zero_mean_normal_density(error, variance):
    """
    Density of a zero-mean normal distribution, prob(a, b^2) in Table 5.2.

    Parameters
    ----------
    error : float or numpy.ndarray
        Value a at which the density is evaluated.
    variance : float or numpy.ndarray
        Variance b^2 of the distribution.

    Returns
    -------
    numpy.ndarray
        Density value(s).
    """
    variance = np.maximum(variance, MIN_VARIANCE)
    return np.exp(-0.5 * error ** 2 / variance) / np.sqrt(2.0 * np.pi * variance)


def sample_zero_mean_normal(variance, size, rng):
    """
    Draw from a zero-mean normal distribution, sample(b^2) in Table 5.4.

    Parameters
    ----------
    variance : float
        Variance b^2 of the distribution.
    size : int
        Number of samples.
    rng : numpy.random.Generator
        Random number generator.

    Returns
    -------
    numpy.ndarray
        Array of shape (size,).
    """
    return rng.normal(0.0, np.sqrt(variance), size)


def velocity_motion_model(pose, control, previous_pose, alphas, dt):
    """
    Closed-form velocity motion model, Table 5.1.

    The function recovers the control (v_hat, w_hat) and the final rotation
    gamma_hat that would move the robot from previous_pose to pose, and
    returns the probability density of the errors between them and the
    commanded control.

    Parameters
    ----------
    pose : array_like
        Candidate pose x_t = (x [m], y [m], yaw [rad]), shape (3,) or (3, ...).
    control : tuple of float
        Commanded control (v [m/s], w [rad/s]).
    previous_pose : array_like
        Pose x_{t-1} = (x [m], y [m], yaw [rad]), shape (3,).
    alphas : tuple of float
        Noise parameters alpha1 to alpha6.
    dt : float
        Duration of the control [s].

    Returns
    -------
    numpy.ndarray
        Density over the noise variables (v, w, gamma), shape pose.shape[1:].
    """
    x, y, yaw = previous_pose
    x_new, y_new, yaw_new = np.asarray(pose, dtype=float)
    speed, turn_rate = control

    # Motion in the frame of the previous pose
    forward = (x_new - x) * np.cos(yaw) + (y_new - y) * np.sin(yaw)
    lateral = (y_new - y) * np.cos(yaw) - (x_new - x) * np.sin(yaw)
    is_straight = np.abs(lateral) < MIN_LATERAL_OFFSET
    safe_lateral = np.where(is_straight, 1.0, lateral)

    # Table 5.1 lines 2 to 4: center of the circle through both poses
    mu = 0.5 * forward / safe_lateral
    center_x = (x + x_new) / 2.0 + mu * (y - y_new)
    center_y = (y + y_new) / 2.0 + mu * (x_new - x)

    # Line 6: angle traveled on the circle
    delta_heading = normalize_angle(np.arctan2(y_new - center_y, x_new - center_x)
                                    - np.arctan2(y - center_y, x - center_x))

    # Lines 5 and 7 use the unsigned radius |r*|, which gives the wrong sign
    # of v_hat when v and w have opposite signs. The signed radius lambda of
    # equation 5.17 (positive for a center on the left of the robot) fixes it.
    signed_radius = (center_y - y) * np.cos(yaw) - (center_x - x) * np.sin(yaw)

    turn_rate_hat = np.where(is_straight, 0.0, delta_heading / dt)  # line 8
    speed_hat = np.where(is_straight, forward / dt,
                         turn_rate_hat * signed_radius)  # line 7, signed
    final_rotation_hat = normalize_angle(
        yaw_new - yaw - turn_rate_hat * dt) / dt  # line 9

    alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = alphas
    return (zero_mean_normal_density(speed - speed_hat,
                                     alpha1 * speed ** 2 + alpha2 * turn_rate ** 2)
            * zero_mean_normal_density(turn_rate - turn_rate_hat,
                                       alpha3 * speed ** 2 + alpha4 * turn_rate ** 2)
            * zero_mean_normal_density(final_rotation_hat,
                                       alpha5 * speed ** 2 + alpha6 * turn_rate ** 2))


def sample_velocity_motion_model(control, previous_pose, alphas, dt,
                                 number_of_samples, rng):
    """
    Sample poses from the velocity motion model, Table 5.3.

    Parameters
    ----------
    control : tuple of float
        Commanded control (v [m/s], w [rad/s]).
    previous_pose : array_like
        Pose x_{t-1} = (x [m], y [m], yaw [rad]), shape (3,).
    alphas : tuple of float
        Noise parameters alpha1 to alpha6.
    dt : float
        Duration of the control [s].
    number_of_samples : int
        Number of poses to draw.
    rng : numpy.random.Generator
        Random number generator.

    Returns
    -------
    numpy.ndarray
        Sampled poses, shape (3, number_of_samples).
    """
    x, y, yaw = previous_pose
    speed, turn_rate = control
    alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = alphas

    speed_hat = speed + sample_zero_mean_normal(
        alpha1 * speed ** 2 + alpha2 * turn_rate ** 2, number_of_samples, rng)
    turn_rate_hat = turn_rate + sample_zero_mean_normal(
        alpha3 * speed ** 2 + alpha4 * turn_rate ** 2, number_of_samples, rng)
    final_rotation_hat = sample_zero_mean_normal(
        alpha5 * speed ** 2 + alpha6 * turn_rate ** 2, number_of_samples, rng)

    # Lines 5 and 6 divide by w_hat. For w_hat close to zero, use the limit of
    # straight motion instead.
    is_straight = np.abs(turn_rate_hat) < 1e-9
    safe_turn_rate = np.where(is_straight, 1.0, turn_rate_hat)
    radius = speed_hat / safe_turn_rate
    new_yaw = yaw + turn_rate_hat * dt
    x_new = np.where(is_straight, x + speed_hat * dt * np.cos(yaw),
                     x - radius * np.sin(yaw) + radius * np.sin(new_yaw))
    y_new = np.where(is_straight, y + speed_hat * dt * np.sin(yaw),
                     y + radius * np.cos(yaw) - radius * np.cos(new_yaw))
    yaw_new = normalize_angle(new_yaw + final_rotation_hat * dt)
    return np.vstack((x_new, y_new, yaw_new))


def odometry_motion_components(start_pose, end_pose):
    """
    Decompose a motion into rotation, translation, rotation (Table 5.5).

    Parameters
    ----------
    start_pose : array_like
        Pose (x [m], y [m], yaw [rad]), shape (3,) or (3, ...).
    end_pose : array_like
        Pose (x [m], y [m], yaw [rad]), shape (3,) or (3, ...).

    Returns
    -------
    tuple of numpy.ndarray
        (rotation1 [rad], translation [m], rotation2 [rad]).
    """
    x0, y0, yaw0 = start_pose
    x1, y1, yaw1 = end_pose
    translation = np.hypot(x1 - x0, y1 - y0)
    # For a very short translation the direction of travel is undefined, so
    # the whole turn goes into the second rotation.
    rotation1 = np.where(translation < MIN_TRANSLATION, 0.0,
                         normalize_angle(np.arctan2(y1 - y0, x1 - x0) - yaw0))
    rotation2 = normalize_angle(yaw1 - yaw0 - rotation1)
    return rotation1, translation, rotation2


def odometry_motion_model(pose, odometry, previous_pose, alphas):
    """
    Closed-form odometry motion model, Table 5.5.

    Parameters
    ----------
    pose : array_like
        Candidate pose x_t = (x [m], y [m], yaw [rad]), shape (3,) or (3, ...).
    odometry : tuple of array_like
        Odometry poses at t-1 and at t, each (x [m], y [m], yaw [rad]).
    previous_pose : array_like
        Pose x_{t-1} = (x [m], y [m], yaw [rad]), shape (3,).
    alphas : tuple of float
        Noise parameters alpha1 to alpha4.

    Returns
    -------
    numpy.ndarray
        Density over the noise variables (rotation1, translation, rotation2),
        shape pose.shape[1:]. To get a density over the pose (x, y, yaw),
        divide by the translation of the candidate pose (the Jacobian).
    """
    alpha1, alpha2, alpha3, alpha4 = alphas
    rotation1, translation, rotation2 = odometry_motion_components(*odometry)
    rotation1_hat, translation_hat, rotation2_hat = odometry_motion_components(
        previous_pose, np.asarray(pose, dtype=float))

    p1 = zero_mean_normal_density(
        normalize_angle(rotation1 - rotation1_hat),
        alpha1 * rotation1_hat ** 2 + alpha2 * translation_hat ** 2)
    p2 = zero_mean_normal_density(
        translation - translation_hat,
        alpha3 * translation_hat ** 2
        + alpha4 * (rotation1_hat ** 2 + rotation2_hat ** 2))
    p3 = zero_mean_normal_density(
        normalize_angle(rotation2 - rotation2_hat),
        alpha1 * rotation2_hat ** 2 + alpha2 * translation_hat ** 2)
    return p1 * p2 * p3


def sample_odometry_motion_model(odometry, previous_pose, alphas,
                                 number_of_samples, rng):
    """
    Sample poses from the odometry motion model, Table 5.6.

    Parameters
    ----------
    odometry : tuple of array_like
        Odometry poses at t-1 and at t, each (x [m], y [m], yaw [rad]).
    previous_pose : array_like
        Pose x_{t-1} = (x [m], y [m], yaw [rad]), shape (3,).
    alphas : tuple of float
        Noise parameters alpha1 to alpha4.
    number_of_samples : int
        Number of poses to draw.
    rng : numpy.random.Generator
        Random number generator.

    Returns
    -------
    numpy.ndarray
        Sampled poses, shape (3, number_of_samples).
    """
    alpha1, alpha2, alpha3, alpha4 = alphas
    rotation1, translation, rotation2 = odometry_motion_components(*odometry)

    rotation1_hat = rotation1 - sample_zero_mean_normal(
        alpha1 * rotation1 ** 2 + alpha2 * translation ** 2,
        number_of_samples, rng)
    translation_hat = translation - sample_zero_mean_normal(
        alpha3 * translation ** 2 + alpha4 * (rotation1 ** 2 + rotation2 ** 2),
        number_of_samples, rng)
    rotation2_hat = rotation2 - sample_zero_mean_normal(
        alpha1 * rotation2 ** 2 + alpha2 * translation ** 2,
        number_of_samples, rng)

    x, y, yaw = previous_pose
    x_new = x + translation_hat * np.cos(yaw + rotation1_hat)
    y_new = y + translation_hat * np.sin(yaw + rotation1_hat)
    yaw_new = normalize_angle(yaw + rotation1_hat + rotation2_hat)
    return np.vstack((x_new, y_new, yaw_new))


def noise_free_velocity_motion(control, previous_pose, dt):
    """
    End pose of the velocity model without noise.

    Parameters
    ----------
    control : tuple of float
        Control (v [m/s], w [rad/s]).
    previous_pose : array_like
        Pose (x [m], y [m], yaw [rad]), shape (3,).
    dt : float
        Duration of the control [s].

    Returns
    -------
    numpy.ndarray
        End pose, shape (3,).
    """
    no_noise = (0.0,) * 6
    return sample_velocity_motion_model(control, previous_pose, no_noise, dt,
                                        1, np.random.default_rng())[:, 0]


def pose_grid_around(samples):
    """
    Regular grid of candidate poses that covers a cloud of samples.

    Parameters
    ----------
    samples : numpy.ndarray
        Sampled poses, shape (3, n).

    Returns
    -------
    tuple
        (x_values, y_values, yaw_values, grid) with grid of shape (3, nx, ny, nyaw).
    """
    margin = 0.3  # [m]
    x_values = np.linspace(samples[0].min() - margin, samples[0].max() + margin,
                           GRID_RESOLUTION)
    y_values = np.linspace(samples[1].min() - margin, samples[1].max() + margin,
                           GRID_RESOLUTION)
    yaw_values = np.linspace(-np.pi, np.pi, YAW_RESOLUTION, endpoint=False)
    grid = np.stack(np.meshgrid(x_values, y_values, yaw_values, indexing="ij"))
    return x_values, y_values, yaw_values, grid


def density_level_for_mass(density, mass):
    """
    Density value whose super-level set holds the given probability mass.

    Parameters
    ----------
    density : numpy.ndarray
        Density values on a regular grid.
    mass : float
        Probability mass between 0 and 1.

    Returns
    -------
    float
        Contour level.
    """
    sorted_values = np.sort(density.ravel())[::-1]
    cumulative = np.cumsum(sorted_values) / sorted_values.sum()
    return sorted_values[np.searchsorted(cumulative, mass)]


def plot_motion(axis, title, samples, density_xy, x_values, y_values,
                start_pose):
    """Draw the samples and the contour of the closed-form density."""
    axis.cla()
    level = density_level_for_mass(density_xy, PROBABILITY_MASS_LEVEL)
    axis.imshow(density_xy.T, origin="lower", cmap="Greys",
                extent=(x_values[0], x_values[-1], y_values[0], y_values[-1]))
    axis.scatter(samples[0], samples[1], s=2, c="tab:red", alpha=0.4)
    axis.contour(x_values, y_values, density_xy.T, levels=[level],
                 colors="tab:blue", linewidths=1.5)
    axis.arrow(start_pose[0], start_pose[1], 0.3 * np.cos(start_pose[2]),
               0.3 * np.sin(start_pose[2]), color="k", width=0.02)
    axis.plot(start_pose[0], start_pose[1], "ks")
    axis.set_title(title, fontsize=10)
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_aspect("equal")


def main():
    print(__file__ + " start!!")
    rng = np.random.default_rng(0)
    start_pose = np.array([0.0, 0.0, 0.0])
    controls = {
        "straight": (1.0, 0.0),
        "left turn": (1.0, 0.35),
        "right turn": (1.0, -0.35),
    }

    if show_animation:
        fig, axes = plt.subplots(1, 2, figsize=(11, 5.5))
        fig.subplots_adjust(top=0.82, bottom=0.2, wspace=0.25)
        fig.legend(handles=[
            plt.Line2D([], [], color="tab:red", marker=".", linestyle="",
                       label="poses drawn by the sampler"),
            plt.Line2D([], [], color="tab:blue",
                       label=f"{PROBABILITY_MASS_LEVEL:.0%} region of the "
                             "closed-form density (gray)"),
            plt.Line2D([], [], color="k", marker="s", linestyle="",
                       label="start pose")],
            loc="lower center", ncol=3, fontsize=9)
        # for stopping simulation with the esc key.
        fig.canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

    for name, control in controls.items():
        # The same motion seen by both models: the odometry reports the
        # noise-free end pose of the velocity model.
        odometry = (start_pose,
                    noise_free_velocity_motion(control, start_pose, TIME_STEP))

        velocity_samples = sample_velocity_motion_model(
            control, start_pose, VELOCITY_MODEL_ALPHAS, TIME_STEP,
            NUMBER_OF_SAMPLES, rng)
        odometry_samples = sample_odometry_motion_model(
            odometry, start_pose, ODOMETRY_MODEL_ALPHAS, NUMBER_OF_SAMPLES, rng)

        if show_animation:
            # One grid for both panels, so that the two clouds are comparable
            x_values, y_values, yaw_values, grid = pose_grid_around(
                np.hstack((velocity_samples, odometry_samples)))
            yaw_step = yaw_values[1] - yaw_values[0]
            velocity_density = velocity_motion_model(
                grid, control, start_pose, VELOCITY_MODEL_ALPHAS, TIME_STEP)
            odometry_density = odometry_motion_model(
                grid, odometry, start_pose, ODOMETRY_MODEL_ALPHAS)
            plot_motion(axes[0], f"Velocity model, {name}:\n"
                        f"v = {control[0]} m/s, w = {control[1]} rad/s,"
                        f" dt = {TIME_STEP} s",
                        velocity_samples, velocity_density.sum(axis=2) * yaw_step,
                        x_values, y_values, start_pose)
            plot_motion(axes[1], f"Odometry model, {name}:\n"
                        "odometry reports the noise-free end pose",
                        odometry_samples, odometry_density.sum(axis=2) * yaw_step,
                        x_values, y_values, start_pose)
            fig.suptitle("Probabilistic motion models: sampler and closed form")
            plt.pause(2.0)

    if show_animation:
        plt.show()


if __name__ == '__main__':
    main()
