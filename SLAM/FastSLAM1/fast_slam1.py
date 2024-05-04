"""

FastSLAM 1.0 example

author: Atsushi Sakai (@Atsushi_twi)

"""

import math

import matplotlib.pyplot as plt
import numpy as np
from utils.angle import angle_mod

# Fast SLAM covariance
Q = np.diag([3.0, np.deg2rad(10.0)]) ** 2
R = np.diag([1.0, np.deg2rad(20.0)]) ** 2

#  Simulation parameter
Q_SIM = np.diag([0.3, np.deg2rad(2.0)]) ** 2
R_SIM = np.diag([0.5, np.deg2rad(10.0)]) ** 2
OFFSET_YAW_RATE_NOISE = 0.01

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]
N_PARTICLE = 100  # number of particle
NTH = N_PARTICLE / 1.5  # Number of particle for re-sampling

show_animation = True


class Particle:

    def __init__(self, n_landmark):
        self.w = 1.0 / N_PARTICLE
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # landmark x-y positions
        self.lm = np.zeros((n_landmark, LM_SIZE))
        # landmark position covariance
        self.lmP = np.zeros((n_landmark * LM_SIZE, LM_SIZE))


def fast_slam1(particles, u, z):
    particles = predict_particles(particles, u)

    particles = update_with_observation(particles, z)

    particles = resampling(particles)

    return particles


def normalize_weight(particles):
    sum_w = sum([p.w for p in particles])

    try:
        for i in range(N_PARTICLE):
            particles[i].w /= sum_w
    except ZeroDivisionError:
        for i in range(N_PARTICLE):
            particles[i].w = 1.0 / N_PARTICLE

        return particles

    return particles


def calc_final_state(particles):
    x_est = np.zeros((STATE_SIZE, 1))

    particles = normalize_weight(particles)

    for i in range(N_PARTICLE):
        x_est[0, 0] += particles[i].w * particles[i].x
        x_est[1, 0] += particles[i].w * particles[i].y
        x_est[2, 0] += particles[i].w * particles[i].yaw

    x_est[2, 0] = pi_2_pi(x_est[2, 0])

    return x_est


def predict_particles(particles, u):
    for i in range(N_PARTICLE):
        px = np.zeros((STATE_SIZE, 1))
        px[0, 0] = particles[i].x
        px[1, 0] = particles[i].y
        px[2, 0] = particles[i].yaw
        ud = u + (np.random.randn(1, 2) @ R ** 0.5).T  # add noise
        px = motion_model(px, ud)
        particles[i].x = px[0, 0]
        particles[i].y = px[1, 0]
        particles[i].yaw = px[2, 0]

    return particles


def add_new_landmark(particle, z, Q_cov):
    r = z[0]
    b = z[1]
    lm_id = int(z[2])

    s = math.sin(pi_2_pi(particle.yaw + b))
    c = math.cos(pi_2_pi(particle.yaw + b))

    particle.lm[lm_id, 0] = particle.x + r * c
    particle.lm[lm_id, 1] = particle.y + r * s

    # covariance
    dx = r * c
    dy = r * s
    d2 = dx**2 + dy**2
    d = math.sqrt(d2)
    Gz = np.array([[dx / d, dy / d],
                   [-dy / d2, dx / d2]])
    particle.lmP[2 * lm_id:2 * lm_id + 2] = np.linalg.inv(
        Gz) @ Q_cov @ np.linalg.inv(Gz.T)

    return particle


def compute_jacobians(particle, xf, Pf, Q_cov):
    dx = xf[0, 0] - particle.x
    dy = xf[1, 0] - particle.y
    d2 = dx ** 2 + dy ** 2
    d = math.sqrt(d2)

    zp = np.array(
        [d, pi_2_pi(math.atan2(dy, dx) - particle.yaw)]).reshape(2, 1)

    Hv = np.array([[-dx / d, -dy / d, 0.0],
                   [dy / d2, -dx / d2, -1.0]])

    Hf = np.array([[dx / d, dy / d],
                   [-dy / d2, dx / d2]])

    Sf = Hf @ Pf @ Hf.T + Q_cov

    return zp, Hv, Hf, Sf


def update_kf_with_cholesky(xf, Pf, v, Q_cov, Hf):
    PHt = Pf @ Hf.T
    S = Hf @ PHt + Q_cov

    S = (S + S.T) * 0.5
    s_chol = np.linalg.cholesky(S).T
    s_chol_inv = np.linalg.inv(s_chol)
    W1 = PHt @ s_chol_inv
    W = W1 @ s_chol_inv.T

    x = xf + W @ v
    P = Pf - W1 @ W1.T

    return x, P


def update_landmark(particle, z, Q_cov):
    lm_id = int(z[2])
    xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
    Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2, :])

    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q)

    dz = z[0:2].reshape(2, 1) - zp
    dz[1, 0] = pi_2_pi(dz[1, 0])

    xf, Pf = update_kf_with_cholesky(xf, Pf, dz, Q_cov, Hf)

    particle.lm[lm_id, :] = xf.T
    particle.lmP[2 * lm_id:2 * lm_id + 2, :] = Pf

    return particle


def compute_weight(particle, z, Q_cov):
    lm_id = int(z[2])
    xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
    Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2])
    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q_cov)

    dx = z[0:2].reshape(2, 1) - zp
    dx[1, 0] = pi_2_pi(dx[1, 0])

    try:
        invS = np.linalg.inv(Sf)
    except np.linalg.linalg.LinAlgError:
        print("singular")
        return 1.0

    num = np.exp(-0.5 * (dx.T @ invS @ dx))[0, 0]
    den = 2.0 * math.pi * math.sqrt(np.linalg.det(Sf))

    w = num / den

    return w


def update_with_observation(particles, z):
    for iz in range(len(z[0, :])):

        landmark_id = int(z[2, iz])

        for ip in range(N_PARTICLE):
            # new landmark
            if abs(particles[ip].lm[landmark_id, 0]) <= 0.01:
                particles[ip] = add_new_landmark(particles[ip], z[:, iz], Q)
            # known landmark
            else:
                w = compute_weight(particles[ip], z[:, iz], Q)
                particles[ip].w *= w
                particles[ip] = update_landmark(particles[ip], z[:, iz], Q)

    return particles


def resampling(particles):
    """
    low variance re-sampling
    """

    particles = normalize_weight(particles)

    pw = []
    for i in range(N_PARTICLE):
        pw.append(particles[i].w)

    pw = np.array(pw)

    n_eff = 1.0 / (pw @ pw.T)  # Effective particle number

    if n_eff < NTH:  # resampling
        w_cum = np.cumsum(pw)
        base = np.cumsum(pw * 0.0 + 1 / N_PARTICLE) - 1 / N_PARTICLE
        resample_id = base + np.random.rand(base.shape[0]) / N_PARTICLE

        indexes = []
        index = 0
        for ip in range(N_PARTICLE):
            while (index < w_cum.shape[0] - 1) \
                    and (resample_id[ip] > w_cum[index]):
                index += 1
            indexes.append(index)

        tmp_particles = particles[:]
        for i in range(len(indexes)):
            particles[i].x = tmp_particles[indexes[i]].x
            particles[i].y = tmp_particles[indexes[i]].y
            particles[i].yaw = tmp_particles[indexes[i]].yaw
            particles[i].lm = tmp_particles[indexes[i]].lm[:, :]
            particles[i].lmP = tmp_particles[indexes[i]].lmP[:, :]
            particles[i].w = 1.0 / N_PARTICLE

    return particles


def calc_input(time):
    if time <= 3.0:  # wait at first
        v = 0.0
        yaw_rate = 0.0
    else:
        v = 1.0  # [m/s]
        yaw_rate = 0.1  # [rad/s]

    u = np.array([v, yaw_rate]).reshape(2, 1)

    return u


def observation(x_true, xd, u, rfid):
    # calc true state
    x_true = motion_model(x_true, u)

    # add noise to range observation
    z = np.zeros((3, 0))
    for i in range(len(rfid[:, 0])):

        dx = rfid[i, 0] - x_true[0, 0]
        dy = rfid[i, 1] - x_true[1, 0]
        d = math.hypot(dx, dy)
        angle = pi_2_pi(math.atan2(dy, dx) - x_true[2, 0])
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Q_SIM[0, 0] ** 0.5  # add noise
            angle_with_noize = angle + np.random.randn() * Q_SIM[
                1, 1] ** 0.5  # add noise
            zi = np.array([dn, pi_2_pi(angle_with_noize), i]).reshape(3, 1)
            z = np.hstack((z, zi))

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * R_SIM[0, 0] ** 0.5
    ud2 = u[1, 0] + np.random.randn() * R_SIM[
        1, 1] ** 0.5 + OFFSET_YAW_RATE_NOISE
    ud = np.array([ud1, ud2]).reshape(2, 1)

    xd = motion_model(xd, ud)

    return x_true, z, xd, ud


def motion_model(x, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = F @ x + B @ u

    x[2, 0] = pi_2_pi(x[2, 0])

    return x


def pi_2_pi(angle):
    return angle_mod(angle)


def main():
    print(__file__ + " start!!")

    time = 0.0

    # RFID positions [x, y]
    rfid = np.array([[10.0, -2.0],
                     [15.0, 10.0],
                     [15.0, 15.0],
                     [10.0, 20.0],
                     [3.0, 15.0],
                     [-5.0, 20.0],
                     [-5.0, 5.0],
                     [-10.0, 15.0]
                     ])
    n_landmark = rfid.shape[0]

    # State Vector [x y yaw v]'
    x_est = np.zeros((STATE_SIZE, 1))  # SLAM estimation
    x_true = np.zeros((STATE_SIZE, 1))  # True state
    x_dr = np.zeros((STATE_SIZE, 1))  # Dead reckoning

    # history
    hist_x_est = x_est
    hist_x_true = x_true
    hist_x_dr = x_dr

    particles = [Particle(n_landmark) for _ in range(N_PARTICLE)]

    while SIM_TIME >= time:
        time += DT
        u = calc_input(time)

        x_true, z, x_dr, ud = observation(x_true, x_dr, u, rfid)

        particles = fast_slam1(particles, ud, z)

        x_est = calc_final_state(particles)

        x_state = x_est[0: STATE_SIZE]

        # store data history
        hist_x_est = np.hstack((hist_x_est, x_state))
        hist_x_dr = np.hstack((hist_x_dr, x_dr))
        hist_x_true = np.hstack((hist_x_true, x_true))

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event', lambda event:
                [exit(0) if event.key == 'escape' else None])
            plt.plot(rfid[:, 0], rfid[:, 1], "*k")

            for i in range(N_PARTICLE):
                plt.plot(particles[i].x, particles[i].y, ".r")
                plt.plot(particles[i].lm[:, 0], particles[i].lm[:, 1], "xb")

            plt.plot(hist_x_true[0, :], hist_x_true[1, :], "-b")
            plt.plot(hist_x_dr[0, :], hist_x_dr[1, :], "-k")
            plt.plot(hist_x_est[0, :], hist_x_est[1, :], "-r")
            plt.plot(x_est[0], x_est[1], "xk")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
