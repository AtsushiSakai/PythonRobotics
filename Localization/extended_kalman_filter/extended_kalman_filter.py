"""

Extended kalman filter (EKF) localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import matplotlib.pyplot as plt

# Estimation parameter of EKF
Q = np.diag([0.1, 0.1, math.radians(1.0), 1.0])**2
R = np.diag([2.0, math.radians(40.0)])**2

#  Simulation parameter
Qsim = np.diag([0.5, 0.5])**2
Rsim = np.diag([1.0, math.radians(30.0)])**2

DT = 0.1  # time tick [s]
SIM_TIME = 60.0  # simulation time [s]


#  function ShowErrorEllipse(xEst,PEst)
#  %誤差分散円を計算し、表示する関数
#  Pxy=PEst(1:2,1:2);%x,yの共分散を取得
#  [eigvec, eigval]=eig(Pxy);%固有値と固有ベクトルの計算
#  %固有値の大きい方のインデックスを探す
#  if eigval(1,1)>=eigval(2,2)
#  bigind=1;
#  smallind=2;
#  else
#  bigind=2;
#  smallind=1;
#  end

#  chi=9.21;%誤差楕円のカイの二乗分布値　99%

#  %楕円描写
#  t=0:10:360;
#  a=sqrt(eigval(bigind,bigind)*chi);
#  b=sqrt(eigval(smallind,smallind)*chi);
#  x=[a*cosd(t);
#  b*sind(t)];
#  %誤差楕円の角度を計算
#  angle = atan2(eigvec(bigind,2),eigvec(bigind,1));
#  if(angle < 0)
#  angle = angle + 2*pi;
#  end

#  %誤差楕円の回転
#  R=[cos(angle) sin(angle);
#  -sin(angle) cos(angle)];
#  x=R*x;
#  plot(x(1,:)+xEst(1),x(2,:)+xEst(2))


def do_control():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]

    u = np.matrix([v, yawrate]).T

    return u


def observation(xTrue, xd, u):

    xTrue = motion_model(xTrue, u)

    zx = xTrue[0, 0] + np.random.randn() * Qsim[0, 0]
    zy = xTrue[1, 0] + np.random.randn() * Qsim[1, 1]
    z = np.matrix([zx, zy])

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


def jacobF(x, u):
    # Jacobian of Motion Model
    yaw = x[2, 0]
    u1 = u[0, 0]
    jF = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [-DT * u1 * math.sin(yaw), DT * u1 * math.cos(yaw), 1, 0],
        [DT * math.cos(yaw), DT * math.sin(yaw), 0, 1]])

    return jF


def jacobH(x):
    # Jacobian of Observation Model
    jH = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH


def ekf_estimation(xEst, PEst, z, u):

    #  Predict
    xPred = motion_model(xEst, u)
    jF = jacobF(xPred, u)
    PPred = jF * PEst * jF.T + Q

    #  Update
    jH = jacobH(xPred)
    zPred = observation_model(xPred)
    y = z.T - zPred
    S = jH * PPred * jH.T + R
    K = PPred * jH.T * np.linalg.inv(S)
    xEst = xPred + K * y
    PEst = (np.eye(len(xEst)) - K * jH) * PPred

    return xEst, PEst


def main():
    print(__file__ + " start!!")

    time = 0.0
    # State Vector [x y yaw v]'
    xEst = np.matrix(np.zeros((4, 1)))
    xTrue = np.matrix(np.zeros((4, 1)))
    PEst = np.eye(4)

    # Dead Reckoning
    xDR = np.matrix(np.zeros((4, 1)))

    while SIM_TIME >= time:
        time += DT
        u = do_control()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u)

        xEst, PEst = ekf_estimation(xEst, PEst, z, ud)

        plt.plot(xTrue[0, 0], xTrue[1, 0], ".b")
        plt.plot(xDR[0, 0], xDR[1, 0], ".k")
        plt.plot(z[0, 0], z[0, 1], ".g")
        plt.plot(xEst[0, 0], xEst[1, 0], ".r")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)


if __name__ == '__main__':
    main()
