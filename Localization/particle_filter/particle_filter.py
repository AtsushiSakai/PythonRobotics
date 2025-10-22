"""
Particle Filter Localization Sample

This module implements Monte Carlo Localization (MCL) using a particle filter.
The particle filter is a non-parametric implementation of the Bayes filter
that represents the belief by a set of random state samples (particles).

Key Features:
- Non-parametric: Can represent multi-modal distributions
- Handles non-linear motion and observation models
- Uses importance sampling and resampling

Algorithm Overview:
1. Prediction: Propagate particles using motion model with noise
2. Update: Weight particles based on observation likelihood
3. Resampling: Resample particles based on weights to avoid degeneracy

author: Atsushi Sakai (@Atsushi_twi)
"""
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import math

import matplotlib.pyplot as plt
import numpy as np

from utils.angle import rot_mat_2d

# Estimation parameters for Particle Filter
Q = np.diag([0.2]) ** 2  # Observation noise covariance: range measurement error [m^2]
R = np.diag([2.0, np.deg2rad(40.0)]) ** 2  # Process noise covariance: [velocity, yaw_rate] error

# Simulation parameters (ground truth noise)
Q_sim = np.diag([0.2]) ** 2  # Simulation observation noise [m^2]
R_sim = np.diag([1.0, np.deg2rad(30.0)]) ** 2  # Simulation process noise

DT = 0.1  # Time step [s]
SIM_TIME = 50.0  # Total simulation time [s]
MAX_RANGE = 20.0  # Maximum observation range for RFID sensors [m]

# Particle filter parameters
NP = 100  # Number of particles - more particles = better accuracy but slower
NTh = NP / 2.0  # Resampling threshold based on effective particle number

show_animation = True


def calc_input():
    """
    Calculate control input for the robot.

    Returns:
        u: Control input vector [velocity, yaw_rate]^T
           - velocity: forward velocity [m/s]
           - yaw_rate: angular velocity [rad/s]
    """
    v = 1.0  # Forward velocity [m/s]
    yaw_rate = 0.1  # Angular velocity (yaw rate) [rad/s]
    u = np.array([[v, yaw_rate]]).T
    return u


def observation(x_true, xd, u, rf_id):
    """
    Simulate observation and dead reckoning with noise.

    This function simulates:
    1. Ground truth motion with perfect control input
    2. Range observations to RFID landmarks with noise
    3. Dead reckoning with noisy control input

    Args:
        x_true: True state vector [x, y, yaw, v]^T
        xd: Dead reckoning state vector
        u: Control input [v, yaw_rate]^T
        rf_id: RFID landmark positions [[x1, y1], [x2, y2], ...]

    Returns:
        x_true: Updated true state
        z: Observations [range, landmark_x, landmark_y] for visible landmarks
        xd: Updated dead reckoning state
        ud: Noisy control input used for dead reckoning
    """
    # Update true state with perfect control input
    x_true = motion_model(x_true, u)

    # Generate range observations to RFID landmarks
    z = np.zeros((0, 3))  # Initialize empty observation array

    for i in range(len(rf_id[:, 0])):
        # Calculate distance from robot to landmark
        dx = x_true[0, 0] - rf_id[i, 0]
        dy = x_true[1, 0] - rf_id[i, 1]
        d = math.hypot(dx, dy)

        # Only observe landmarks within sensor range
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Q_sim[0, 0] ** 0.5  # Add Gaussian noise to range
            zi = np.array([[dn, rf_id[i, 0], rf_id[i, 1]]])
            z = np.vstack((z, zi))

    # Add noise to control input for dead reckoning simulation
    ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0] ** 0.5  # Noisy velocity
    ud2 = u[1, 0] + np.random.randn() * R_sim[1, 1] ** 0.5  # Noisy yaw rate
    ud = np.array([[ud1, ud2]]).T

    # Update dead reckoning estimate using noisy input
    xd = motion_model(xd, ud)

    return x_true, z, xd, ud


def motion_model(x, u):
    """
    Motion model for robot kinematics.

    Implements a simple 2D kinematic model:
        x' = x + v*cos(yaw)*dt
        y' = y + v*sin(yaw)*dt
        yaw' = yaw + yaw_rate*dt
        v' = v

    The model is expressed as: x_{t+1} = F*x_t + B*u_t

    Args:
        x: State vector [x, y, yaw, v]^T
           - x, y: 2D position [m]
           - yaw: orientation [rad]
           - v: velocity [m/s]
        u: Control input [v, yaw_rate]^T

    Returns:
        x: Next state vector [x, y, yaw, v]^T
    """
    # State transition matrix (identity for position/orientation, zero for velocity)
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    # Control input matrix (converts velocity and yaw_rate to state changes)
    B = np.array([[DT * math.cos(x[2, 0]), 0],      # x position change
                  [DT * math.sin(x[2, 0]), 0],      # y position change
                  [0.0, DT],                         # yaw change
                  [1.0, 0.0]])                       # velocity update

    # Apply motion model: x = F*x + B*u
    x = F.dot(x) + B.dot(u)

    return x


def gauss_likelihood(x, sigma):
    """
    Calculate Gaussian likelihood (probability density).

    This is used to compute the importance weight of each particle
    based on how well its predicted observation matches the actual observation.

    Formula: p(x) = (1/sqrt(2*pi*sigma^2)) * exp(-x^2 / (2*sigma^2))

    Args:
        x: Innovation (difference between predicted and actual observation)
        sigma: Standard deviation of observation noise

    Returns:
        p: Likelihood (probability density) of the observation
    """
    p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2) * \
        math.exp(-x ** 2 / (2 * sigma ** 2))

    return p


def calc_covariance(x_est, px, pw):
    """
    Calculate covariance matrix from weighted particles.

    The covariance is computed using the weighted sample covariance formula
    with Bessel's correction for weighted samples:

    Cov = (1/(1-sum(w_i^2))) * sum(w_i * (x_i - x_mean) * (x_i - x_mean)^T)

    This accounts for the effective sample size of weighted particles.

    Args:
        x_est: Estimated state (weighted mean of particles) [4x1]
        px: Particle states [4xNP] where each column is a particle
        pw: Particle weights [1xNP], should sum to 1.0

    Returns:
        cov: Covariance matrix [4x4] representing uncertainty in the estimate
    """
    cov = np.zeros((4, 4))
    n_particle = px.shape[1]

    # Weighted covariance calculation
    for i in range(n_particle):
        dx = (px[:, i:i + 1] - x_est)  # Deviation from mean
        cov += pw[0, i] * dx @ dx.T     # Weighted outer product

    # Bessel's correction for weighted samples
    cov *= 1.0 / (1.0 - pw @ pw.T)

    return cov


def pf_localization(px, pw, z, u):
    """
    Particle Filter Localization - Main algorithm.

    This implements the standard particle filter algorithm:
    1. Prediction Step: Propagate each particle using motion model with noise
    2. Update Step: Weight particles based on observation likelihood
    3. Resampling Step: Resample if effective particle number is too low

    Algorithm Steps:
    - For each particle:
        - Sample noisy control input from motion noise distribution
        - Predict new particle state using motion model
        - Calculate importance weight based on observation likelihood
    - Normalize weights
    - Estimate state as weighted mean of particles
    - Resample if particle degeneracy detected (low N_eff)

    Args:
        px: Particle states [4xNP], each column represents one particle [x,y,yaw,v]^T
        pw: Particle weights [1xNP], importance weights summing to 1.0
        z: Observations [Mx3] where M is number of visible landmarks
           Each row: [range, landmark_x, landmark_y]
        u: Control input [v, yaw_rate]^T

    Returns:
        x_est: Estimated state [4x1] - weighted mean of particles
        p_est: Estimated covariance [4x4] - uncertainty of estimate
        px: Updated particle states (after potential resampling)
        pw: Updated particle weights (after potential resampling)
    """

    # ========== Prediction and Update Step ==========
    for ip in range(NP):
        x = np.array([px[:, ip]]).T  # Get particle state
        w = pw[0, ip]                 # Get particle weight

        # Prediction: Add process noise to control input (motion model uncertainty)
        ud1 = u[0, 0] + np.random.randn() * R[0, 0] ** 0.5  # Noisy velocity
        ud2 = u[1, 0] + np.random.randn() * R[1, 1] ** 0.5  # Noisy yaw rate
        ud = np.array([[ud1, ud2]]).T
        x = motion_model(x, ud)  # Propagate particle

        # Update: Calculate importance weight based on observations
        for i in range(len(z[:, 0])):
            # Predict observation: distance from particle to landmark
            dx = x[0, 0] - z[i, 1]  # x difference to landmark
            dy = x[1, 0] - z[i, 2]  # y difference to landmark
            pre_z = math.hypot(dx, dy)  # Predicted range

            # Innovation: difference between predicted and actual observation
            dz = pre_z - z[i, 0]

            # Update weight by multiplying likelihoods from all observations
            w = w * gauss_likelihood(dz, math.sqrt(Q[0, 0]))

        # Store updated particle state and weight
        px[:, ip] = x[:, 0]
        pw[0, ip] = w

    # ========== Normalization ==========
    pw = pw / pw.sum()  # Normalize weights to sum to 1.0

    # ========== State Estimation ==========
    x_est = px.dot(pw.T)  # Weighted mean (expected value)
    p_est = calc_covariance(x_est, px, pw)  # Weighted covariance

    # ========== Resampling Step ==========
    # Calculate effective particle number: N_eff = 1 / sum(w_i^2)
    # If N_eff is low, particles are degenerating (few particles have significant weight)
    N_eff = 1.0 / (pw.dot(pw.T))[0, 0]
    if N_eff < NTh:
        px, pw = re_sampling(px, pw)

    return x_est, p_est, px, pw


def re_sampling(px, pw):
    """
    Low Variance Resampling (Systematic Resampling).

    This is a more efficient resampling method than naive multinomial resampling.
    It reduces the variance of the resampling process by using stratified sampling.

    Algorithm:
    1. Create cumulative sum of weights (CDF)
    2. Generate equally-spaced points with random offset
    3. For each point, find corresponding particle in CDF
    4. Sample particles proportional to their weights

    Advantages over multinomial resampling:
    - Lower variance: particles with similar weights are selected more consistently
    - Computational efficiency: O(N) instead of O(N log N)
    - Better preservation of particle diversity

    Args:
        px: Particle states [4xNP]
        pw: Particle weights [1xNP]

    Returns:
        px: Resampled particle states
        pw: Reset weights (uniform: 1/NP for all particles)
    """

    # Create cumulative distribution function (CDF) from weights
    w_cum = np.cumsum(pw)

    # Generate systematic sample points: evenly spaced with random offset
    base = np.arange(0.0, 1.0, 1 / NP)  # Evenly spaced points [0, 1/N, 2/N, ...]
    re_sample_id = base + np.random.uniform(0, 1 / NP)  # Add random offset [0, 1/N)

    # Select particles based on CDF
    indexes = []
    ind = 0
    for ip in range(NP):
        # Find first particle whose CDF value exceeds the sample point
        while re_sample_id[ip] > w_cum[ind]:
            ind += 1
        indexes.append(ind)

    # Create new particle set from selected indices
    px = px[:, indexes]
    pw = np.zeros((1, NP)) + 1.0 / NP  # Reset to uniform weights

    return px, pw


def plot_covariance_ellipse(x_est, p_est):  # pragma: no cover
    """
    Plot covariance ellipse representing uncertainty in position estimate.

    The ellipse is drawn using eigenvalue decomposition of the position covariance:
    - Eigenvalues determine the size (length of semi-axes)
    - Eigenvectors determine the orientation

    This visualizes the 1-sigma uncertainty region in x-y position.

    Args:
        x_est: Estimated state [4x1]
        p_est: Estimated covariance matrix [4x4]
    """
    # Extract x-y position covariance
    p_xy = p_est[0:2, 0:2]

    # Eigenvalue decomposition to find ellipse parameters
    eig_val, eig_vec = np.linalg.eig(p_xy)

    # Determine major and minor axes
    if eig_val[0] >= eig_val[1]:
        big_ind = 0
        small_ind = 1
    else:
        big_ind = 1
        small_ind = 0

    # Parameter for ellipse boundary
    t = np.arange(0, 2 * math.pi + 0.1, 0.1)

    # Handle numerical errors: eigenvalues should be positive but can be
    # slightly negative (~10^-20) due to numerical precision
    try:
        a = math.sqrt(eig_val[big_ind])  # Semi-major axis
    except ValueError:
        a = 0

    try:
        b = math.sqrt(eig_val[small_ind])  # Semi-minor axis
    except ValueError:
        b = 0

    # Generate ellipse points in canonical form
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]

    # Rotate ellipse to align with covariance eigenvectors
    angle = math.atan2(eig_vec[1, big_ind], eig_vec[0, big_ind])
    fx = rot_mat_2d(angle) @ np.array([[x, y]])

    # Translate ellipse to estimated position
    px = np.array(fx[:, 0] + x_est[0, 0]).flatten()
    py = np.array(fx[:, 1] + x_est[1, 0]).flatten()
    plt.plot(px, py, "--r")


def main():
    """
    Main simulation loop for particle filter localization.

    Simulation scenario:
    - Robot moves in 2D space with noisy velocity and yaw rate
    - RFID landmarks provide noisy range measurements
    - Three estimators are compared:
        1. Ground truth (perfect motion)
        2. Dead reckoning (noisy odometry only)
        3. Particle filter (fusion of odometry and range measurements)
    """
    print(__file__ + " start!!")

    time = 0.0

    # RFID landmark positions [x, y] in global frame
    # These are known landmark positions used for localization
    rf_id = np.array([[10.0, 0.0],
                      [10.0, 10.0],
                      [0.0, 15.0],
                      [-5.0, 20.0]])

    # Initialize state vectors [x, y, yaw, v]^T
    x_est = np.zeros((4, 1))   # Particle filter estimate
    x_true = np.zeros((4, 1))  # Ground truth state
    x_dr = np.zeros((4, 1))    # Dead reckoning estimate

    # Initialize particle filter
    px = np.zeros((4, NP))  # Particle states: each column is one particle
    pw = np.zeros((1, NP)) + 1.0 / NP  # Uniform initial weights

    # History for plotting trajectories
    h_x_est = x_est
    h_x_true = x_true
    h_x_dr = x_true

    # Main simulation loop
    while SIM_TIME >= time:
        time += DT
        u = calc_input()  # Get control input

        # Simulate one time step
        x_true, z, x_dr, ud = observation(x_true, x_dr, u, rf_id)

        # Run particle filter localization
        x_est, PEst, px, pw = pf_localization(px, pw, z, ud)

        # Store trajectory history
        h_x_est = np.hstack((h_x_est, x_est))
        h_x_dr = np.hstack((h_x_dr, x_dr))
        h_x_true = np.hstack((h_x_true, x_true))

        if show_animation:
            plt.cla()
            # Enable stopping simulation with the ESC key
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            # Plot observations (lines from robot to observed landmarks)
            for i in range(len(z[:, 0])):
                plt.plot([x_true[0, 0], z[i, 1]], [x_true[1, 0], z[i, 2]], "-k")

            # Plot landmarks
            plt.plot(rf_id[:, 0], rf_id[:, 1], "*k", markersize=10, label="Landmarks")

            # Plot particles (red dots show particle distribution)
            plt.plot(px[0, :], px[1, :], ".r", alpha=0.5, label="Particles")

            # Plot trajectories
            plt.plot(np.array(h_x_true[0, :]).flatten(),
                     np.array(h_x_true[1, :]).flatten(), "-b", label="Ground Truth")
            plt.plot(np.array(h_x_dr[0, :]).flatten(),
                     np.array(h_x_dr[1, :]).flatten(), "-k", label="Dead Reckoning")
            plt.plot(np.array(h_x_est[0, :]).flatten(),
                     np.array(h_x_est[1, :]).flatten(), "-r", label="Particle Filter")

            # Plot uncertainty ellipse
            plot_covariance_ellipse(x_est, PEst)

            plt.axis("equal")
            plt.grid(True)
            plt.legend()
            plt.pause(0.001)


if __name__ == '__main__':
    main()
