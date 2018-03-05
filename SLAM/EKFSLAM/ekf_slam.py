"""

Extended Kalman Filter based SLAM example

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import matplotlib.pyplot as plt

Cx = np.diag([1.0, 1.0, math.radians(30.0)])**2
Cz = np.diag([1.0, 1.0])**2


#  Simulation parameter
Qsim = np.diag([0.2])**2
Rsim = np.diag([1.0, math.radians(30.0)])**2

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range

M_DIST_TH = 1.0

POSE_SIZE = 3  # [x,y,yaw]
LM_SIZE = 2  # [x,y]

# Particle filter parameter
NP = 100  # Number of Particle
NTh = NP / 2.0  # Number of particle for re-sampling

show_animation = True


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

        dx = xTrue[0, 0] - RFID[i, 0]
        dy = xTrue[1, 0] - RFID[i, 1]
        d = math.sqrt(dx**2 + dy**2)
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Qsim[0, 0]  # add noise
            zi = np.matrix([dn, RFID[i, 0], RFID[i, 1]])
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
    n = int((len(x) - POSE_SIZE) / LM_SIZE)
    return n


def jacob_motion(x, u):

    Fx = np.hstack((np.eye(POSE_SIZE), np.zeros(
        (POSE_SIZE, LM_SIZE * calc_n_LM(x)))))

    jF = np.matrix([[0.0, 0.0, -DT * u[0] * math.sin(x[2, 0])],
                    [0.0, 0.0, DT * u[0] * math.cos(x[2, 0])],
                    [0.0, 0.0, 0.0]])

    G = np.eye(POSE_SIZE) + Fx.T * jF * Fx

    return G, Fx,


def calc_LM_Pos(x, z):

    zp = np.zeros((2, 1))

    zp[0, 0] = x[0, 0] + z[0, 0] * math.cos(x[2, 0] + z[0, 1])
    zp[1, 0] = x[1, 0] + z[0, 0] * math.sin(x[2, 0] + z[0, 1])

    return zp


def calc_mahalanobis_dist(xAug, PAug, z, iz):

    nLM = calc_n_LM(xAug)

    mdist = []

    for i in range(nLM):

        if i == nLM - 1:
            mdist.append(M_DIST_TH)
        else:
            lm = xAug[POSE_SIZE + LM_SIZE *
                      i: POSE_SIZE + LM_SIZE * (i + 1), :]
            y, S, H = calc_innovation(lm, xAug, PAug, z[iz, 0:2], i)
            mdist.append(y.T * np.linalg.inv(S) * y)

    return mdist


def calc_innovation(lm, xEst, PEst, z, LMid):
    delta = lm - xEst[0:2]
    q = (delta.T * delta)[0, 0]
    zangle = math.atan2(delta[1], delta[0]) - xEst[2]
    zp = [math.sqrt(q), pi_2_pi(zangle)]
    y = (z - zp).T
    H = jacobH(q, delta, xEst, LMid + 1)
    S = H * PEst * H.T + Cz

    return y, S, H


def jacobH(q, delta, x, i):
    sq = math.sqrt(q)
    G = np.matrix([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                   [delta[1, 0], - delta[0, 0], - 1.0, - delta[1, 0], delta[0, 0]]])

    G = G / q
    nLM = calc_n_LM(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

    F = np.vstack((F1, F2))

    H = G * F

    return H


def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def ekf_slam(xEst, PEst, u, z):
    # Predict
    xEst[0:POSE_SIZE] = motion_model(xEst[0:POSE_SIZE], u)
    G, Fx = jacob_motion(xEst[0:POSE_SIZE], u)
    PEst[0:POSE_SIZE, 0:POSE_SIZE] = G.T * \
        PEst[0:POSE_SIZE, 0:POSE_SIZE] * G + Fx.T * Cx * Fx
    initP = np.eye(2) * 1000.0

    # Update
    for iz in range(len(z[:, 0])):  # for each observation
        #  print(iz)

        zp = calc_LM_Pos(xEst, z[iz, :])

        # Extend state and covariance matrix
        xAug = np.vstack((xEst, zp))
        PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
                          np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))

        mah_dists = calc_mahalanobis_dist(xAug, PAug, z, iz)
        minid = mah_dists.index(min(mah_dists))
        #  print(minid)

        nLM = calc_n_LM(xAug)
        if minid == (nLM - 1):
            print("New LM")
            xEst = xAug
            PEst = PAug
        else:
            print("Old LM")

        lm = xEst[POSE_SIZE + LM_SIZE *
                  iz: POSE_SIZE + LM_SIZE * (iz + 1), :]
        y, S, H = calc_innovation(lm, xEst, PEst, z[iz, 0:2], iz)

        K = PEst * H.T * np.linalg.inv(S)
        xEst = xEst + K * y
        PEst = (np.eye(len(xEst)) - K * H) * PEst

    xEst[2] = pi_2_pi(xEst[2])

    return xEst, PEst


def main():
    print(__file__ + " start!!")

    time = 0.0

    # RFID positions [x, y]
    RFID = np.array([[10.0, 0.0],
                     [10.0, 10.0],
                     [0.0, 15.0],
                     [-5.0, 20.0]])

    # State Vector [x y yaw v]'
    xEst = np.matrix(np.zeros((POSE_SIZE, 1)))
    xTrue = np.matrix(np.zeros((POSE_SIZE, 1)))
    PEst = np.eye(POSE_SIZE)

    xDR = np.matrix(np.zeros((POSE_SIZE, 1)))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        xEst, PEst = ekf_slam(xEst, PEst, ud, z)

        x_state = xEst[0:POSE_SIZE]

        # store data history
        hxEst = np.hstack((hxEst, x_state))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        if show_animation:
            plt.cla()

            for i in range(len(z[:, 0])):
                plt.plot([xTrue[0, 0], z[i, 1]], [xTrue[1, 0], z[i, 2]], "-k")
            plt.plot(RFID[:, 0], RFID[:, 1], "*k")
            plt.plot(xEst[0], xEst[1], ".r")
            plt.plot(np.array(hxTrue[0, :]).flatten(),
                     np.array(hxTrue[1, :]).flatten(), "-b")
            plt.plot(np.array(hxDR[0, :]).flatten(),
                     np.array(hxDR[1, :]).flatten(), "-k")
            plt.plot(np.array(hxEst[0, :]).flatten(),
                     np.array(hxEst[1, :]).flatten(), "-r")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
