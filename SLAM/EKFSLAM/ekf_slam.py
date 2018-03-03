"""

Extended Kalman Filter based SLAM example

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import matplotlib.pyplot as plt

Cx = np.diag([1.0, 1.0, math.radians(30.0)])**2

#  Simulation parameter
Qsim = np.diag([0.2])**2
Rsim = np.diag([1.0, math.radians(30.0)])**2

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range

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


def ekf_slam(xEst, PEst, u, z):
    #  Predict
    xEst = motion_model(xEst, u)
    G, Fx = jacob_motion(xEst, u)
    PEst = G.T * PEst * G + Fx.T * Cx * Fx

    return xEst, PEst

    #  % Update
    #  for iz=1:length(z(:,1))%それぞれの観測値に対して
    #  %観測値をランドマークとして追加
    #  zl=CalcLMPosiFromZ(xEst,z(iz,:));%観測値そのものからLMの位置を計算
    #  %状態ベクトルと共分散行列の追加
    #  xAug=[xEst;zl];
    #  PAug=[PEst zeros(length(xEst),LMSize);
    #  zeros(LMSize,length(xEst)) initP];

    #  mdist=[];%マハラノビス距離のリスト
    #  for il=1:GetnLM(xAug) %それぞれのランドマークについて
    #  if il==GetnLM(xAug)
    #  mdist=[mdist alpha];%新しく追加した点の距離はパラメータ値を使う
    #  else
    #  lm=xAug(4+2*(il-1):5+2*(il-1));
    #  [y,S,H]=CalcInnovation(lm,xAug,PAug,z(iz,1:2),il);
    #  mdist=[mdist y'*inv(S)*y];%マハラノビス距離の計算
    #  end
    #  end

    #  %マハラノビス距離が最も近いものに対応付け
    #  [C,I]=min(mdist);

    #  %一番距離が小さいものが追加したものならば、その観測値をランドマークとして採用
    #  if I==GetnLM(xAug)
    #  %disp('New LM')
    #  xEst=xAug;
    #  PEst=PAug;
    #  end

    #  lm=xEst(4+2*(I-1):5+2*(I-1));%対応付けられたランドマークデータの取得
    #  %イノベーションの計算
    #  [y,S,H]=CalcInnovation(lm,xEst,PEst,z(iz,1:2),I);
    #  K = PEst*H'*inv(S);
    #  xEst = xEst + K*y;
    #  PEst = (eye(size(xEst,1)) - K*H)*PEst;
    #  end

    #  xEst(3)=PI2PI(xEst(3));%角度補正


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

        # store data history
        hxEst = np.hstack((hxEst, xEst))
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
