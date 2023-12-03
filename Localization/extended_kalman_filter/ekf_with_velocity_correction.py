"""

Extended kalman filter (EKF) localization with velocity correction sample

author: Atsushi Sakai (@Atsushi_twi)
modified by: Ryohei Sasaki (@rsasaki0109)

"""
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import math
import matplotlib.pyplot as plt
import numpy as np

from utils.plot import plot_covariance_ellipse

# Covariance for EKF simulation
Q = np.diag([
    0.1,  # variance of location on x-axis
    0.1,  # variance of location on y-axis
    np.deg2rad(1.0),  # variance of yaw angle
    0.4,  # variance of velocity
    0.1  # variance of scale factor
]) ** 2  # predict state covariance
R = np.diag([0.1, 0.1]) ** 2  # Observation x,y position covariance

#  Simulation parameter
INPUT_NOISE = np.diag([0.1, np.deg2rad(5.0)]) ** 2
GPS_NOISE = np.diag([0.05, 0.05]) ** 2

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]

show_animation = True


def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.array([[v], [yawrate]])
    return u


def observation(xTrue, xd, u):
    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = observation_model(xTrue) + GPS_NOISE @ np.random.randn(2, 1)

    # add noise to input
    ud = u + INPUT_NOISE @ np.random.randn(2, 1)

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0, 0],
                  [0, 1.0, 0, 0, 0],
                  [0, 0, 1.0, 0, 0],
                  [0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]) * x[4, 0], 0],
                  [DT * math.sin(x[2, 0]) * x[4, 0], 0],
                  [0.0, DT],
                  [1.0, 0.0],
                  [0.0, 0.0]])

    x = F @ x + B @ u

    return x


def observation_model(x):
    H = np.array([
        [1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0]
    ])
    z = H @ x

    return z


def jacob_f(x, u):
    """
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v*s*dt*cos(yaw)
    y_{t+1} = y_t+v*s*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    s_{t+1} = s{t}
    so
    dx/dyaw = -v*s*dt*sin(yaw)
    dx/dv = dt*s*cos(yaw)
    dx/ds = dt*v*cos(yaw)
    dy/dyaw = v*s*dt*cos(yaw)
    dy/dv = dt*s*sin(yaw)
    dy/ds = dt*v*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    s = x[4, 0]
    jF = np.array([
        [1.0, 0.0, -DT * v * s * math.sin(yaw), DT * s * math.cos(yaw), DT * v * math.cos(yaw)],
        [0.0, 1.0, DT * v * s * math.cos(yaw), DT * s * math.sin(yaw), DT * v * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0]])
    return jF


def jacob_h():
    jH = np.array([[1, 0, 0, 0, 0],
                   [0, 1, 0, 0, 0]])
    return jH


def ekf_estimation(xEst, PEst, z, u):
    #  Predict
    xPred = motion_model(xEst, u)
    jF = jacob_f(xEst, u)
    PPred = jF @ PEst @ jF.T + Q

    #  Update
    jH = jacob_h()
    zPred = observation_model(xPred)
    y = z - zPred
    S = jH @ PPred @ jH.T + R
    K = PPred @ jH.T @ np.linalg.inv(S)
    xEst = xPred + K @ y
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
    return xEst, PEst


def main():
    print(__file__ + " start!!")

    time = 0.0

    #  State Vector [x y yaw v s]'
    xEst = np.zeros((5, 1))
    xEst[4, 0] = 1.0 #  Initial scale factor
    xTrue = np.zeros((5, 1))
    true_scale_factor = 0.9 #  True scale factor
    xTrue[4, 0] = true_scale_factor
    PEst = np.eye(5)

    xDR = np.zeros((5, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((2, 1))

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u)

        xEst, PEst = ekf_estimation(xEst, PEst, z, ud)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.hstack((hz, z))
        estimated_scale_factor = hxEst[4, -1]
        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(hz[0, :], hz[1, :], ".g")
            plt.plot(hxTrue[0, :].flatten(),
                     hxTrue[1, :].flatten(), "-b")
            plt.plot(hxDR[0, :].flatten(),
                     hxDR[1, :].flatten(), "-k")
            plt.plot(hxEst[0, :].flatten(),
                     hxEst[1, :].flatten(), "-r")
            plt.text(0.45, 0.85, f"True Velocity Scale Factor: {true_scale_factor:.2f}", ha='left', va='top', transform=plt.gca().transAxes)
            plt.text(0.45, 0.95, f"Estimated Velocity Scale Factor: {estimated_scale_factor:.2f}", ha='left', va='top', transform=plt.gca().transAxes)
            plot_covariance_ellipse(xEst[0, 0], xEst[1, 0], PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
