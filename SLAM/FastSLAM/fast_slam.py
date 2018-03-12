"""

Fast SLAM example

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import matplotlib.pyplot as plt


# EKF state covariance
Cx = np.diag([0.5, 0.5, math.radians(30.0)])**2


#  Simulation parameter
Qsim = np.diag([0.2, math.radians(1.0)])**2
Rsim = np.diag([1.0, math.radians(10.0)])**2

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM srate size [x,y]

N_PARTICLE = 100  # number of particle

NTH = N_PARTICLE / 2.0  # Number of particle for re-sampling

show_animation = True


class Particle:

    def __init__(self, N_LM):
        self.w = 1.0 / N_PARTICLE
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.lm = np.zeros((N_LM, 2))
        self.lmP = [np.zeros((2, 2))] * N_LM


def normalize_weight(particles):

    sumw = sum([particles[ip].w for ip in range(N_PARTICLE)])

    #  if sumw <= 0.0000001:
    #  for i in range(N_PARTICLE):
    #  particles[i].w = 1.0 / N_PARTICLE
    #  return particles

    for i in range(N_PARTICLE):
        particles[i].w = particles[i].w / sumw

    return particles


def calc_final_state(particles):

    xEst = np.zeros((STATE_SIZE, 1))

    for i in range(N_PARTICLE):
        xEst[0, 0] += particles[i].w * particles[i].x
        xEst[1, 0] += particles[i].w * particles[i].y
        xEst[2, 0] += particles[i].w * particles[i].yaw

    xEst[2, 0] = pi_2_pi(xEst[2, 0])

    return xEst


def predict_particles(particles, u):

    for i in range(N_PARTICLE):
        px = np.zeros((STATE_SIZE, 1))
        px[0, 0] = particles[i].x
        px[1, 0] = particles[i].y
        px[2, 0] = particles[i].yaw
        ud = u + np.matrix(np.random.randn(1, 2)) * Rsim  # add noise
        px = motion_model(px, ud)
        particles[i].x = px[0, 0]
        particles[i].y = px[1, 0]
        particles[i].yaw = px[2, 0]

    return particles


def add_new_lm(particle, z):

    r = z[0, 0]
    b = z[0, 1]
    lm_id = int(z[0, 2])

    s = math.sin(particle.yaw + b)
    c = math.cos(particle.yaw + b)

    particle.lm[lm_id, 0] = particle.x + r * c
    particle.lm[lm_id, 1] = particle.y + r * s

    # covariance
    Gz = np.matrix([[c, -r * s],
                    [s, r * c]])

    particle.lmP[lm_id] = Gz * Cx[0:2, 0:2] * Gz.T

    return particle


def compute_weight(particle, z):

    lm_id = int(z[0, 2])

    lmxy = np.matrix(particle.lm[lm_id, :])

    # calc landmark xy
    r = z[0, 0]
    b = z[0, 1]
    lm_id = int(z[0, 2])

    s = math.sin(particle.yaw + b)
    c = math.cos(particle.yaw + b)

    zxy = np.zeros((1, 2))

    zxy[0, 0] = particle.x + r * c
    zxy[0, 1] = particle.y + r * s

    dx = (lmxy - zxy).T
    S = particle.lmP[lm_id]

    num = math.exp(-0.5 * dx.T * np.linalg.inv(S) * dx)
    den = 2.0 * math.pi * math.sqrt(np.linalg.det(S))

    w = num / den

    return w


def update_with_observation(particles, z):

    for iz in range(len(z[:, 0])):

        lmid = int(z[iz, 2])

        for ip in range(N_PARTICLE):
            # new landmark
            if abs(particles[ip].lm[lmid, 0]) <= 0.1:
                particles[ip] = add_new_lm(particles[ip], z[iz, :])
            # known landmark
            else:
                w = compute_weight(particles[ip], z[iz, :])  # w = p(z_k | x_k)
                particles[ip].w = particles[ip].w + w
                #  particles(i)= feature_update(particles(i), zf, idf, R)

    return particles


def resampling(particles):
    """
    low variance re-sampling
    """

    particles = normalize_weight(particles)

    pw = []
    for i in range(N_PARTICLE):
        pw.append(particles[i].w)

    pw = np.matrix(pw)
    #  print(pw)

    Neff = 1.0 / (pw * pw.T)[0, 0]  # Effective particle number
    #  print(Neff)

    if Neff < NTH:  # resampling
        print("resamping")
        wcum = np.cumsum(pw)
        base = np.cumsum(pw * 0.0 + 1 / N_PARTICLE) - 1 / N_PARTICLE
        resampleid = base + np.random.rand(base.shape[1]) / N_PARTICLE

        inds = []
        ind = 0
        for ip in range(N_PARTICLE):
            while ((ind < wcum.shape[1] - 1) and (resampleid[0, ip] > wcum[0, ind])):
                ind += 1
            inds.append(ind)

        tparticles = particles[:]
        for i in range(len(inds)):
            particles[i] = tparticles[inds[i]]

        particles = normalize_weight(particles)

    return particles


def fast_slam(particles, PEst, u, z):

    # Predict
    particles = predict_particles(particles, u)

    # Observation
    particles = update_with_observation(particles, z)

    particles = normalize_weight(particles)

    particles = resampling(particles)

    xEst = calc_final_state(particles)

    return xEst, PEst


def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.matrix([v, yawrate]).T
    return u


def observation(xTrue, xd, u, RFID):

    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.matrix(np.zeros((0, 3)))

    for i in range(len(RFID[:, 0])):

        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.sqrt(dx**2 + dy**2)
        angle = pi_2_pi(math.atan2(dy, dx))
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Qsim[0, 0]  # add noise
            anglen = angle + np.random.randn() * Qsim[1, 1]  # add noise
            zi = np.matrix([dn, anglen, i])
            z = np.vstack((z, zi))

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1]
    ud = np.matrix([ud1, ud2]).T

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):

    F = np.matrix([[1.0, 0, 0],
                   [0, 1.0, 0],
                   [0, 0, 1.0]])

    B = np.matrix([[DT * math.cos(x[2, 0]), 0],
                   [DT * math.sin(x[2, 0]), 0],
                   [0.0, DT]])

    x = F * x + B * u

    return x


def calc_n_LM(x):
    n = int((len(x) - STATE_SIZE) / LM_SIZE)
    return n


def calc_LM_Pos(x, z):

    zp = np.zeros((2, 1))

    zp[0, 0] = x[0, 0] + z[0, 0] * math.cos(x[2, 0] + z[0, 1])
    zp[1, 0] = x[1, 0] + z[0, 0] * math.sin(x[2, 0] + z[0, 1])

    return zp


def get_LM_Pos_from_state(x, ind):

    lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

    return lm


def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def main():
    print(__file__ + " start!!")

    time = 0.0

    # RFID positions [x, y]
    RFID = np.array([[10.0, -2.0],
                     [15.0, 10.0],
                     [3.0, 15.0],
                     [-5.0, 20.0]])
    N_LM = RFID.shape[0]

    # State Vector [x y yaw v]'
    xEst = np.matrix(np.zeros((STATE_SIZE, 1)))
    xTrue = np.matrix(np.zeros((STATE_SIZE, 1)))
    PEst = np.eye(STATE_SIZE)

    xDR = np.matrix(np.zeros((STATE_SIZE, 1)))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue

    particles = [Particle(N_LM) for i in range(N_PARTICLE)]

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        xEst, PEst = fast_slam(particles, PEst, ud, z)

        x_state = xEst[0:STATE_SIZE]

        # store data history
        hxEst = np.hstack((hxEst, x_state))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        if show_animation:
            plt.cla()

            plt.plot(RFID[:, 0], RFID[:, 1], "*k")

            for i in range(N_PARTICLE):
                plt.plot(particles[i].x, particles[i].y, ".r")

            # plot landmark
            for i in range(calc_n_LM(xEst)):
                plt.plot(xEst[STATE_SIZE + i * 2],
                         xEst[STATE_SIZE + i * 2 + 1], "xg")

            plt.plot(np.array(hxTrue[0, :]).flatten(),
                     np.array(hxTrue[1, :]).flatten(), "-b")
            plt.plot(np.array(hxDR[0, :]).flatten(),
                     np.array(hxDR[1, :]).flatten(), "-k")
            plt.plot(np.array(hxEst[0, :]).flatten(),
                     np.array(hxEst[1, :]).flatten(), "-r")

            plt.plot(xEst[0], xEst[1], "xk")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
