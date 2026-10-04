"""Planar GPS/IMU fusion with an extended Kalman filter.

State: [x, y, vx, vy, yaw, accelerometer_bias_x, accelerometer_bias_y,
        gyroscope_bias]. Positions and velocities are in a local metric world
frame. IMU acceleration is in the body frame, with gravity already removed.
Only horizontal motion is modeled; roll, pitch, altitude, magnetometers and
barometers are outside this example. GPS and IMU are assumed time-aligned and
co-located. Initial heading and velocity are assumed approximately known.

Run this file to compare fusion with and without bias estimation against
IMU-only dead reckoning, including a GPS outage. Without bias estimation, the
state contains only [x, y, vx, vy, yaw] and biases are assumed zero.
No external data or dependencies beyond NumPy/Matplotlib are needed.

Reference: Oliver J. Woodman, An introduction to inertial navigation, 2007.
https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf
"""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


DT = 0.05  # IMU sample period [s] (20 Hz)
GPS_INTERVAL = 20  # one GPS position every 20 IMU samples (1 Hz)
SIM_TIME = 50.0  # [s]
IMU_STD = np.array([0.1, 0.1, np.deg2rad(0.3)])  # [m/s^2, m/s^2, rad/s]
GPS_STD = 0.8  # per-axis position standard deviation [m]
BIAS_RW_STD = np.array([0.002, 0.002, np.deg2rad(0.02)])  # per sqrt(second)
TRUE_BIAS = np.array([0.04, -0.03, np.deg2rad(0.4)])
show_animation = True


def rotation(yaw):
    """Rotate a 2D body-frame vector into the world frame."""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def motion_model(state, imu, dt):
    """Propagate one IMU sample, holding world acceleration constant for dt."""
    bias = state[5:] if len(state) == 8 else np.zeros(3)
    acceleration = rotation(state[4]) @ (imu[:2] - bias[:2])
    predicted = state.copy()
    predicted[:2] += state[2:4] * dt + 0.5 * acceleration * dt**2
    predicted[2:4] += acceleration * dt
    predicted[4] = wrap_angle(state[4] + (imu[2] - bias[2]) * dt)
    return predicted


def motion_jacobians(state, imu, dt):
    """Return state and IMU-input Jacobians of the discrete motion model."""
    rot = rotation(state[4])
    bias = state[5:7] if len(state) == 8 else np.zeros(2)
    acceleration = rot @ (imu[:2] - bias)
    yaw_derivative = np.array([-acceleration[1], acceleration[0]])
    f = np.eye(len(state))
    f[:2, 2:4] = np.eye(2) * dt
    f[:2, 4] = 0.5 * yaw_derivative * dt**2
    f[2:4, 4] = yaw_derivative * dt
    if len(state) == 8:
        f[:2, 5:7] = -0.5 * rot * dt**2
        f[2:4, 5:7] = -rot * dt
        f[4, 7] = -dt

    g = np.zeros((len(state), 3))
    g[:2, :2] = 0.5 * rot * dt**2
    g[2:4, :2] = rot * dt
    g[4, 2] = dt
    return f, g


def predict(state, covariance, imu, dt=DT):
    """Predict at the IMU rate, including sensor noise and bias random walk.

    IMU_STD describes independent noise on each discrete IMU sample, while
    BIAS_RW_STD describes continuous bias random walk per sqrt(second).
    A five-state filter assumes zero bias and has no bias random walk.
    """
    f, g = motion_jacobians(state, imu, dt)
    process_noise = g @ np.diag(IMU_STD**2) @ g.T
    if len(state) == 8:
        process_noise[5:8, 5:8] += np.diag(BIAS_RW_STD**2) * dt
    predicted_covariance = f @ covariance @ f.T + process_noise
    return motion_model(state, imu, dt), (
        predicted_covariance + predicted_covariance.T) / 2.0


def update_gps(state, covariance, position):
    """Correct the predicted state when a new 2D GPS position is available.

    GPS observes only position. Velocity, heading and biases are corrected
    through cross-covariances accumulated during IMU prediction.
    """
    h = np.zeros((2, len(state)))
    h[:, :2] = np.eye(2)
    r = np.eye(2) * GPS_STD**2
    innovation_covariance = h @ covariance @ h.T + r
    gain = np.linalg.solve(innovation_covariance, h @ covariance).T
    updated = state + gain @ (position - h @ state)
    updated[4] = wrap_angle(updated[4])
    # Joseph form preserves covariance symmetry and positive semidefiniteness.
    residual = np.eye(len(state)) - gain @ h
    updated_covariance = residual @ covariance @ residual.T + gain @ r @ gain.T
    return updated, (updated_covariance + updated_covariance.T) / 2.0


def reference_motion(time):
    """Analytic figure-eight truth and ideal body-frame IMU measurements."""
    position = np.array([20.0 * np.sin(0.1 * time), 10.0 * np.sin(0.2 * time)])
    velocity = np.array([2.0 * np.cos(0.1 * time), 2.0 * np.cos(0.2 * time)])
    acceleration = np.array([-0.2 * np.sin(0.1 * time), -0.4 * np.sin(0.2 * time)])
    yaw = np.arctan2(velocity[1], velocity[0])
    yaw_rate = (velocity[0] * acceleration[1] - velocity[1] * acceleration[0]) / (
        velocity @ velocity)
    state = np.concatenate((position, velocity, [yaw], TRUE_BIAS))
    imu = np.concatenate((rotation(yaw).T @ acceleration, [yaw_rate]))
    return state, imu


def simulate(duration=SIM_TIME, seed=0, gps_outage=(20.0, 30.0)):
    """Run a deterministic simulation with multi-rate sensors.

    GPS outages cover [start, end); pass None for uninterrupted GPS. Truth is
    computed analytically, independently of the filter's discrete integrator.
    All three estimates share the same biased, noisy measurements and start
    with the known initial pose/velocity. The eight-state EKF starts with zero
    estimated bias; the five-state EKF and dead reckoning assume zero bias.
    """
    rng = np.random.default_rng(seed)
    times = np.arange(int(round(duration / DT)) + 1) * DT
    truth = np.array([reference_motion(t)[0] for t in times])
    estimates = np.zeros_like(truth)
    no_bias_estimates = np.zeros((len(times), 5))
    dead_reckoning = np.zeros_like(truth)
    covariances = np.zeros((len(times), 8, 8))
    no_bias_covariances = np.zeros((len(times), 5, 5))
    gps = np.full((len(times), 2), np.nan)
    state = truth[0].copy()
    state[5:] = 0.0
    dead_state = state.copy()
    covariance = np.diag([1.0, 1.0, 0.2, 0.2, np.deg2rad(5.0),
                          0.1, 0.1, np.deg2rad(1.0)])**2
    no_bias_state = state[:5].copy()
    no_bias_covariance = covariance[:5, :5].copy()
    estimates[0], dead_reckoning[0], covariances[0] = state, dead_state, covariance
    no_bias_estimates[0], no_bias_covariances[0] = no_bias_state, no_bias_covariance
    for i in range(1, len(times)):
        _, ideal_imu = reference_motion(times[i - 1])
        imu = ideal_imu + TRUE_BIAS + rng.normal(size=3) * IMU_STD
        state, covariance = predict(state, covariance, imu)
        no_bias_state, no_bias_covariance = predict(no_bias_state, no_bias_covariance, imu)
        dead_state = motion_model(dead_state, imu, DT)
        # Draw every scheduled fix, even during outages, to keep sensor noise
        # identical when comparing different outage schedules with the same seed.
        if i % GPS_INTERVAL == 0:
            measurement = truth[i, :2] + rng.normal(size=2) * GPS_STD
            if gps_outage is None or not gps_outage[0] <= times[i] < gps_outage[1]:
                gps[i] = measurement
                state, covariance = update_gps(state, covariance, measurement)
                no_bias_state, no_bias_covariance = update_gps(
                    no_bias_state, no_bias_covariance, measurement)
        estimates[i], dead_reckoning[i], covariances[i] = state, dead_state, covariance
        no_bias_estimates[i], no_bias_covariances[i] = no_bias_state, no_bias_covariance
    return {"time": times, "truth": truth, "estimate": estimates,
            "no_bias_estimate": no_bias_estimates, "no_bias_covariance": no_bias_covariances,
            "dead_reckoning": dead_reckoning, "covariance": covariances,
            "gps": gps, "gps_outage": gps_outage}


def create_animation(history):  # pragma: no cover
    """Create the path/error animation; keep the returned object alive to play it."""
    fig, (path_ax, error_ax) = plt.subplots(1, 2, figsize=(11, 4.8))
    colors = {"truth": "black", "estimate": "tab:blue",
              "no_bias_estimate": "tab:red", "dead_reckoning": "tab:orange"}
    labels = {"truth": "Ground truth", "estimate": "EKF (bias estimation)",
              "no_bias_estimate": "EKF (no bias estimation)",
              "dead_reckoning": "IMU only"}
    styles = {"truth": "-", "estimate": "-", "no_bias_estimate": "--", "dead_reckoning": ":"}
    lines = {key: path_ax.plot([], [], color=color, linestyle=styles[key], label=labels[key])[0]
             for key, color in colors.items()}
    gps_line, = path_ax.plot([], [], "+", color="tab:green", label="GPS fixes", alpha=0.7)
    positions = np.vstack([history[key][:, :2] for key in colors])
    path_ax.set(xlim=(positions[:, 0].min() - 3, positions[:, 0].max() + 3),
                ylim=(positions[:, 1].min() - 3, positions[:, 1].max() + 3),
                xlabel="x [m]", ylabel="y [m]")
    path_ax.set_aspect("equal", adjustable="box")
    path_ax.legend(loc="best", fontsize=8)
    errors = {key: np.linalg.norm(history[key][:, :2] - history["truth"][:, :2], axis=1)
              for key in ["estimate", "no_bias_estimate", "dead_reckoning"]}
    error_lines = {key: error_ax.plot([], [], color=colors[key], linestyle=styles[key], label=labels[key])[0]
                   for key in errors}
    error_ax.set(xlim=(0, history["time"][-1]),
                 ylim=(0, max(values.max() for values in errors.values()) * 1.1 + 0.1),
                 xlabel="Time [s]", ylabel="Position error [m]")
    outage = history["gps_outage"]
    if outage is not None:
        error_ax.axvspan(*outage, color="gray", alpha=0.2, label="GPS outage")
    error_ax.legend(loc="upper left", fontsize=8)
    path_ax.grid(True)
    error_ax.grid(True)
    title = fig.suptitle("GPS/IMU fusion")
    fig.tight_layout()

    def update(index):
        for key, line in lines.items():
            line.set_data(history[key][:index + 1, 0], history[key][:index + 1, 1])
        gps_line.set_data(history["gps"][:index + 1, 0], history["gps"][:index + 1, 1])
        for key, line in error_lines.items():
            line.set_data(history["time"][:index + 1], errors[key][:index + 1])
        time = history["time"][index]
        status = "GPS unavailable" if outage is not None and outage[0] <= time < outage[1] else "GPS available (1 Hz)"
        title.set_text(f"GPS/IMU fusion — {time:.1f} s — {status}")
        return [*lines.values(), gps_line, *error_lines.values(), title]

    frames = list(range(0, len(history["time"]), 5))
    if frames[-1] != len(history["time"]) - 1:
        frames.append(len(history["time"]) - 1)
    return FuncAnimation(fig, update, frames=frames, interval=50, repeat=False)


def main():
    history = simulate(duration=SIM_TIME)
    for key in ["estimate", "no_bias_estimate", "dead_reckoning"]:
        error = history[key][:, :2] - history["truth"][:, :2]
        print(f"{key} position RMSE: {np.sqrt(np.mean(np.sum(error**2, axis=1))):.2f} m")
    animation = create_animation(history) if show_animation else None
    if animation is not None:
        plt.show()
    return history


if __name__ == "__main__":
    main()
