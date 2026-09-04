"""
GPS and IMU fusion with a bias-aware extended Kalman filter.

The filter estimates planar position, velocity, yaw, accelerometer bias, and
gyroscope bias.  Accelerometer readings are expressed in the body frame while
GPS measurements are expressed in the world frame.  Bias states are part of
the estimate, so the filter can recover from the slowly varying errors that
otherwise make inertial dead reckoning drift quickly.

The process covariance is constructed from an input-noise covariance and a
bias random-walk spectral density.  GPS updates use the Joseph covariance
form, which keeps the covariance symmetric and positive semidefinite in the
presence of round-off.  An optional normalized-innovation gate can reject
isolated GPS outliers without changing the nominal update path.
"""

import matplotlib.pyplot as plt
import numpy as np


STATE_SIZE = 8
YAW_INDEX = 4
ACCEL_BIAS_SLICE = slice(5, 7)
GYRO_BIAS_INDEX = 7


def wrap_angle(angle):
    """Wrap an angle in radians to the half-open interval ``[-pi, pi)``."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def rotation_matrix(yaw):
    """Return the 2-D body-to-world rotation matrix for ``yaw``."""
    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    return np.array([[cosine, -sine], [sine, cosine]])


def _vector(value, size, name):
    """Convert a measurement to a finite one-dimensional vector."""
    vector = np.asarray(value, dtype=float).reshape(-1)
    if vector.size != size or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain {size} finite values")
    return vector


def _covariance(value, size, name):
    """Validate and symmetrize a covariance matrix."""
    covariance = np.asarray(value, dtype=float)
    if covariance.shape != (size, size) or not np.all(np.isfinite(covariance)):
        raise ValueError(f"{name} must be a finite {size}x{size} matrix")
    covariance = 0.5 * (covariance + covariance.T)
    if np.min(np.linalg.eigvalsh(covariance)) < -1e-12:
        raise ValueError(f"{name} must be positive semidefinite")
    return covariance


def _transition(state, acceleration, gyro, dt):
    """Propagate one nominal state with constant body-frame acceleration."""
    next_state = state.copy()
    yaw = state[YAW_INDEX]
    unbiased_acceleration = acceleration - state[ACCEL_BIAS_SLICE]
    acceleration_world = rotation_matrix(yaw) @ unbiased_acceleration

    next_state[0:2] += state[2:4] * dt + 0.5 * acceleration_world * dt**2
    next_state[2:4] += acceleration_world * dt
    next_state[YAW_INDEX] = wrap_angle(yaw + (gyro - state[GYRO_BIAS_INDEX]) * dt)
    return next_state


def _transition_jacobian(state, acceleration, dt):
    """Return the Jacobian of :func:`_transition` with respect to the state."""
    yaw = state[YAW_INDEX]
    unbiased_acceleration = acceleration - state[ACCEL_BIAS_SLICE]
    rotation = rotation_matrix(yaw)
    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    dacceleration_dyaw = np.array(
        [
            -sine * unbiased_acceleration[0] - cosine * unbiased_acceleration[1],
            cosine * unbiased_acceleration[0] - sine * unbiased_acceleration[1],
        ]
    )

    jacobian = np.eye(STATE_SIZE)
    jacobian[0:2, 2:4] = np.eye(2) * dt
    jacobian[0:2, YAW_INDEX] = 0.5 * dt**2 * dacceleration_dyaw
    jacobian[0:2, ACCEL_BIAS_SLICE] = -0.5 * dt**2 * rotation
    jacobian[2:4, YAW_INDEX] = dt * dacceleration_dyaw
    jacobian[2:4, ACCEL_BIAS_SLICE] = -dt * rotation
    jacobian[YAW_INDEX, GYRO_BIAS_INDEX] = -dt
    return jacobian


class GPSIMUEKF:
    """Fuse planar IMU and GPS measurements with an extended Kalman filter.

    Parameters
    ----------
    dt : float
        Default IMU sample period in seconds.
    state : array-like, optional
        Initial state ``[x, y, vx, vy, yaw, bax, bay, bgz]``.
    covariance : array-like, optional
        Initial 8x8 state covariance.
    imu_noise : array-like, optional
        3x3 covariance for ``[ax, ay, gyro_z]`` measurements.
    gps_noise : array-like, optional
        2x2 covariance for ``[x, y]`` GPS measurements.
    bias_random_walk : array-like, optional
        3x3 continuous-time spectral density for accelerometer and gyro bias
        random walks.  The discrete process contribution scales with ``dt``.
    """

    def __init__(
        self,
        dt=0.01,
        state=None,
        covariance=None,
        imu_noise=None,
        gps_noise=None,
        bias_random_walk=None,
    ):
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be a positive finite value")
        self.dt = float(dt)

        if state is None:
            state = np.zeros(STATE_SIZE)
        self.state = _vector(state, STATE_SIZE, "state")

        if covariance is None:
            covariance = np.diag([4.0, 4.0, 1.0, 1.0, 0.5, 0.04, 0.04, 0.01])
        self.covariance = _covariance(covariance, STATE_SIZE, "covariance")

        if imu_noise is None:
            imu_noise = np.diag([0.06**2, 0.06**2, 0.015**2])
        self.imu_noise = _covariance(imu_noise, 3, "imu_noise")

        if gps_noise is None:
            gps_noise = np.diag([0.6**2, 0.6**2])
        self.gps_noise = _covariance(gps_noise, 2, "gps_noise")

        if bias_random_walk is None:
            bias_random_walk = np.diag([0.002**2, 0.002**2, 0.001**2])
        self.bias_random_walk = _covariance(bias_random_walk, 3, "bias_random_walk")
        self.last_nis = np.nan
        self.last_update_accepted = False

    def _process_covariance(self, yaw, dt):
        """Build the discrete process covariance for one IMU interval."""
        noise_mapping = np.zeros((STATE_SIZE, 6))
        rotation = rotation_matrix(yaw)
        noise_mapping[0:2, 0:2] = 0.5 * dt**2 * rotation
        noise_mapping[2:4, 0:2] = dt * rotation
        noise_mapping[YAW_INDEX, 2] = dt
        noise_mapping[5:8, 3:6] = np.eye(3)

        noise_covariance = np.zeros((6, 6))
        noise_covariance[0:3, 0:3] = self.imu_noise
        noise_covariance[3:6, 3:6] = self.bias_random_walk * dt
        return noise_mapping @ noise_covariance @ noise_mapping.T

    def predict(self, acceleration, gyro, dt=None):
        """Propagate the estimate with one body-frame IMU sample.

        Parameters
        ----------
        acceleration : array-like, shape (2,)
            Measured body-frame acceleration in ``m/s^2``.
        gyro : float
            Measured yaw rate in ``rad/s``.
        dt : float, optional
            Override the filter's default sample period for this step.

        Returns
        -------
        numpy.ndarray
            A copy of the updated state.
        """
        acceleration = _vector(acceleration, 2, "acceleration")
        gyro_vector = _vector([gyro], 1, "gyro")
        if dt is None:
            dt = self.dt
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be a positive finite value")
        dt = float(dt)

        previous_state = self.state.copy()
        jacobian = _transition_jacobian(previous_state, acceleration, dt)
        process_covariance = self._process_covariance(previous_state[YAW_INDEX], dt)
        self.state = _transition(previous_state, acceleration, gyro_vector[0], dt)
        self.covariance = jacobian @ self.covariance @ jacobian.T
        self.covariance += process_covariance
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        return self.state.copy()

    def update_gps(self, position, covariance=None, gate_threshold=None):
        """Apply a GPS position update and return whether it was accepted.

        ``gate_threshold`` is an optional squared Mahalanobis-distance limit.
        A measurement outside the gate is ignored, which prevents a single
        multipath or dropout spike from moving the state estimate.
        """
        position = _vector(position, 2, "position")
        if covariance is None:
            measurement_covariance = self.gps_noise
        else:
            measurement_covariance = _covariance(covariance, 2, "covariance")
        if gate_threshold is not None and (
            not np.isfinite(gate_threshold) or gate_threshold <= 0.0
        ):
            raise ValueError("gate_threshold must be a positive finite value")

        observation_matrix = np.zeros((2, STATE_SIZE))
        observation_matrix[:, 0:2] = np.eye(2)
        residual = position - observation_matrix @ self.state
        innovation_covariance = (
            observation_matrix @ self.covariance @ observation_matrix.T
            + measurement_covariance
        )
        solved_residual = np.linalg.solve(innovation_covariance, residual)
        self.last_nis = float(residual @ solved_residual)
        if gate_threshold is not None and self.last_nis > gate_threshold:
            self.last_update_accepted = False
            return False

        state_covariance_observation = self.covariance @ observation_matrix.T
        kalman_gain = np.linalg.solve(
            innovation_covariance, state_covariance_observation.T
        ).T
        self.state += kalman_gain @ residual
        self.state[YAW_INDEX] = wrap_angle(self.state[YAW_INDEX])

        identity_minus_gain = np.eye(STATE_SIZE) - kalman_gain @ observation_matrix
        self.covariance = (
            identity_minus_gain @ self.covariance @ identity_minus_gain.T
            + kalman_gain @ measurement_covariance @ kalman_gain.T
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.last_update_accepted = True
        return True


def run_simulation(duration=20.0, dt=0.02, gps_period=0.2, seed=7):
    """Run a deterministic curved-trajectory GPS/IMU fusion simulation.

    The returned dictionary contains ``truth``, ``estimate``,
    ``dead_reckoning``, ``gps``, and ``time`` arrays.  GPS rows without a
    measurement are filled with ``NaN`` values so they can be plotted directly.
    """
    if duration <= 0.0 or dt <= 0.0 or gps_period <= 0.0:
        raise ValueError("duration, dt, and gps_period must be positive")
    step_count = int(round(duration / dt))
    gps_stride = max(1, int(round(gps_period / dt)))
    if step_count < 1:
        raise ValueError("duration must contain at least one step")

    rng = np.random.default_rng(seed)
    true_bias = np.array([0.08, -0.05, 0.02])
    true_state = np.zeros(STATE_SIZE)
    true_state[2] = 1.0
    true_state[ACCEL_BIAS_SLICE] = true_bias[0:2]
    true_state[GYRO_BIAS_INDEX] = true_bias[2]
    estimate = GPSIMUEKF(dt=dt)
    dead_reckoning = np.zeros(STATE_SIZE)
    dead_reckoning[2] = 1.0

    truth_history = np.zeros((step_count + 1, STATE_SIZE))
    estimate_history = np.zeros_like(truth_history)
    dead_reckoning_history = np.zeros_like(truth_history)
    gps_history = np.full((step_count + 1, 2), np.nan)
    truth_history[0] = true_state
    estimate_history[0] = estimate.state
    dead_reckoning_history[0] = dead_reckoning

    for step in range(1, step_count + 1):
        time = step * dt
        true_acceleration = np.array(
            [0.25 * np.cos(0.18 * time), 0.08 * np.sin(0.11 * time)]
        )
        true_gyro = 0.12 + 0.08 * np.sin(0.13 * time)
        true_state = _transition(
            true_state,
            true_acceleration + true_bias[0:2],
            true_gyro + true_bias[2],
            dt,
        )
        measured_acceleration = true_acceleration + true_bias[0:2]
        measured_acceleration += rng.normal(0.0, [0.06, 0.06])
        measured_gyro = true_gyro + true_bias[2] + rng.normal(0.0, 0.015)

        estimate.predict(measured_acceleration, measured_gyro)
        dead_reckoning = _transition(
            dead_reckoning, measured_acceleration, measured_gyro, dt
        )
        if step % gps_stride == 0:
            gps_position = true_state[0:2] + rng.normal(0.0, [0.6, 0.6])
            gps_history[step] = gps_position
            estimate.update_gps(gps_position, gate_threshold=16.0)

        truth_history[step] = true_state
        estimate_history[step] = estimate.state
        dead_reckoning_history[step] = dead_reckoning

    return {
        "time": np.arange(step_count + 1) * dt,
        "truth": truth_history,
        "estimate": estimate_history,
        "dead_reckoning": dead_reckoning_history,
        "gps": gps_history,
    }


show_animation = True


def main():
    """Run and optionally plot the GPS/IMU fusion example."""
    data = run_simulation()
    if show_animation:
        plt.plot(data["truth"][:, 0], data["truth"][:, 1], "-b", label="True")
        plt.plot(
            data["dead_reckoning"][:, 0],
            data["dead_reckoning"][:, 1],
            "-k",
            label="Dead reckoning",
        )
        plt.plot(
            data["estimate"][:, 0],
            data["estimate"][:, 1],
            "-r",
            label="GPS/IMU EKF",
        )
        valid_gps = np.isfinite(data["gps"][:, 0])
        plt.plot(
            data["gps"][valid_gps, 0],
            data["gps"][valid_gps, 1],
            ".g",
            label="GPS",
        )
        plt.axis("equal")
        plt.grid(True)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.legend()
        plt.show()
    return data


if __name__ == "__main__":
    main()
