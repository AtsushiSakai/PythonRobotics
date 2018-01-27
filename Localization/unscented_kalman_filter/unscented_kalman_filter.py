"""

Uncented kalman filter (UKF) localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import scipy.linalg
import math
import matplotlib.pyplot as plt

# Estimation parameter of UKF
Q = np.diag([0.1, 0.1, math.radians(1.0), 1.0])**2
R = np.diag([1.0, math.radians(40.0)])**2

#  Simulation parameter
Qsim = np.diag([0.5, 0.5])**2
Rsim = np.diag([1.0, math.radians(30.0)])**2

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]

#  UKF Parameter
ALPHA = 0.001
BETA = 2
KAPPA = 0

show_animation = True


def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.matrix([v, yawrate]).T
    return u


def observation(xTrue, xd, u):

    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    zx = xTrue[0, 0] + np.random.randn() * Qsim[0, 0]
    zy = xTrue[1, 0] + np.random.randn() * Qsim[1, 1]
    z = np.matrix([zx, zy])

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1]
    ud = np.matrix([ud1, ud2]).T

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):

    F = np.matrix([[1.0, 0, 0, 0],
                   [0, 1.0, 0, 0],
                   [0, 0, 1.0, 0],
                   [0, 0, 0, 0]])

    B = np.matrix([[DT * math.cos(x[2, 0]), 0],
                   [DT * math.sin(x[2, 0]), 0],
                   [0.0, DT],
                   [1.0, 0.0]])

    x = F * x + B * u

    return x


def observation_model(x):
    #  Observation Model
    H = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = H * x

    return z


def generate_sigma_points(xEst, PEst, gamma):
    sigma = xEst
    Psqrt = np.matrix(scipy.linalg.sqrtm(PEst))
    n = len(xEst[:, 0])
    # Positive direction
    for i in range(n):
        sigma = np.hstack((sigma, xEst + gamma * Psqrt[:, i]))

    # Negative direction
    for i in range(n):
        sigma = np.hstack((sigma, xEst - gamma * Psqrt[:, i]))

    return sigma


def predict_sigma_motion(sigma, u):
    #  Sigma Points predition with motion model
    for i in range(sigma.shape[1]):
        sigma[:, i] = motion_model(sigma[:, i], u)

    return sigma


def predict_sigma_observation(sigma):
    # Sigma Points predition with observation model
    for i in range(sigma.shape[1]):
        sigma[0:2, i] = observation_model(sigma[:, i])

    sigma = sigma[0:2, :]

    return sigma


def calc_sigma_covariance(x, sigma, wc, Pi):
    nSigma = sigma.shape[1]
    d = sigma - x[0:sigma.shape[0], :]
    P = Pi
    for i in range(nSigma):
        P = P + wc[0, i] * d[:, i] * d[:, i].T
    return P


def calc_pxz(sigma, x, z_sigma, zb, wc):
    #  function P=CalcPxz(sigma,xPred,zSigma,zb,wc)
    nSigma = sigma.shape[1]
    dx = np.matrix(sigma - x)
    dz = np.matrix(z_sigma - zb[0:2, :])
    P = np.matrix(np.zeros((dx.shape[0], dz.shape[0])))
    for i in range(nSigma):
        P = P + wc[0, i] * dx[:, i] * dz[:, i].T

    return P


def ukf_estimation(xEst, PEst, z, u, wm, wc, gamma):

    #  Predict
    sigma = generate_sigma_points(xEst, PEst, gamma)
    sigma = predict_sigma_motion(sigma, u)
    xPred = (wm * sigma.T).T
    PPred = calc_sigma_covariance(xPred, sigma, wc, Q)

    #  Update
    zPred = observation_model(xPred)
    y = z.T - zPred
    sigma = generate_sigma_points(xPred, PPred, gamma)
    zb = (wm * sigma.T).T
    z_sigma = predict_sigma_observation(sigma)
    st = calc_sigma_covariance(zb, z_sigma, wc, R)
    Pxz = calc_pxz(sigma, xPred, z_sigma, zb, wc)
    K = Pxz * np.linalg.inv(st)
    xEst = xPred + K * y
    PEst = PPred - K * st * K.T

    return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.matrix([[math.cos(angle), math.sin(angle)],
                   [-math.sin(angle), math.cos(angle)]])
    fx = R * np.matrix([x, y])
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def setup_ukf(nx):
    lamda = ALPHA ** 2 * (nx + KAPPA) - nx
    # calculate weights
    wm = [lamda / (lamda + nx)]
    wc = [(lamda / (lamda + nx)) + (1 - ALPHA ** 2 + BETA)]
    for i in range(2 * nx):
        wm.append(1.0 / (2 * (nx + lamda)))
        wc.append(1.0 / (2 * (nx + lamda)))
    gamma = math.sqrt(nx + lamda)

    wm = np.matrix(wm)
    wc = np.matrix(wc)

    return wm, wc, gamma


def main():
    print(__file__ + " start!!")

    nx = 4  # State Vector [x y yaw v]'
    xEst = np.matrix(np.zeros((nx, 1)))
    xTrue = np.matrix(np.zeros((nx, 1)))
    PEst = np.eye(nx)
    xDR = np.matrix(np.zeros((nx, 1)))  # Dead reckoning

    wm, wc, gamma = setup_ukf(nx)

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((1, 2))

    time = 0.0

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u)

        xEst, PEst = ukf_estimation(xEst, PEst, z, ud, wm, wc, gamma)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.vstack((hz, z))

        if show_animation:
            plt.cla()
            plt.plot(hz[:, 0], hz[:, 1], ".g")
            plt.plot(np.array(hxTrue[0, :]).flatten(),
                     np.array(hxTrue[1, :]).flatten(), "-b")
            plt.plot(np.array(hxDR[0, :]).flatten(),
                     np.array(hxDR[1, :]).flatten(), "-k")
            plt.plot(np.array(hxEst[0, :]).flatten(),
                     np.array(hxEst[1, :]).flatten(), "-r")
            plot_covariance_ellipse(xEst, PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
