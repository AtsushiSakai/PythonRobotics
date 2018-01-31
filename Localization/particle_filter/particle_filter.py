"""

Particle Filter localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""

#  tic;
#  %movcount=0;
#  % Main loop
#  for i=1 : nSteps
#  time = time + dt;
#  % Input
#  u=doControl(time);
#  % Observation
#  [z,xTrue,xd,u]=Observation(xTrue, xd, u, RFID, MAX_RANGE);

#  % ------ Particle Filter --------
#  for ip=1:NP
#  x=px(:,ip);
#  w=pw(ip);

#  % Dead Reckoning and random sampling
#  x=f(x, u)+sqrt(Q)*randn(3,1);

#  % Calc Inportance Weight
#  for iz=1:length(z(:,1))
#  pz=norm(x(1:2)'-z(iz,2:3));
#  dz=pz-z(iz,1);
#  w=w*Gauss(dz,0,sqrt(R));
#  end
#  px(:,ip)=x;%格納
#  pw(ip)=w;
#  end

#  pw=Normalize(pw,NP);%正規化
#  [px,pw]=Resampling(px,pw,NTh,NP);%リサンプリング
#  xEst=px*pw';%最終推定値は期待値

#  % Simulation Result
#  result.time=[result.time; time];
#  result.xTrue=[result.xTrue; xTrue'];
#  result.xd=[result.xd; xd'];
#  result.xEst=[result.xEst;xEst'];
#  result.u=[result.u; u'];

#  %Animation (remove some flames)
#  if rem(i,5)==0
#  hold off;
#  arrow=0.5;
#  %パーティクル表示
#  for ip=1:NP
#  quiver(px(1,ip),px(2,ip),arrow*cos(px(3,ip)),arrow*sin(px(3,ip)),'ok');hold on;
#  end
#  plot(result.xTrue(:,1),result.xTrue(:,2),'.b');hold on;
#  plot(RFID(:,1),RFID(:,2),'pk','MarkerSize',10);hold on;
#  %観測線の表示
#  if~isempty(z)
#  for iz=1:length(z(:,1))
#  ray=[xTrue(1:2)';z(iz,2:3)];
#  plot(ray(:,1),ray(:,2),'-r');hold on;
#  end
#  end
#  plot(result.xd(:,1),result.xd(:,2),'.k');hold on;
#  plot(result.xEst(:,1),result.xEst(:,2),'.r');hold on;
#  axis equal;
#  grid on;
#  drawnow;
#
#  function [px,pw]=Resampling(px,pw,NTh,NP)
#  %リサンプリングを実施する関数
#  %アルゴリズムはLow Variance Sampling
#  Neff=1.0/(pw*pw');
#  if Neff<NTh %リサンプリング
#  wcum=cumsum(pw);
#  base=cumsum(pw*0+1/NP)-1/NP;%乱数を加える前のbase
#  resampleID=base+rand/NP;%ルーレットを乱数分増やす
#  ppx=px;%データ格納用
#  ind=1;%新しいID
#  for ip=1:NP
#  while(resampleID(ip)>wcum(ind))
#  ind=ind+1;
#  end
#  px(:,ip)=ppx(:,ind);%LVSで選ばれたパーティクルに置き換え
#  pw(ip)=1/NP;%尤度は初期化
#  end
#  end

#  function pw=Normalize(pw,NP)
#  %重みベクトルを正規化する関数
#  sumw=sum(pw);
#  if sumw~=0
#  pw=pw/sum(pw);%正規化
#  else
#  pw=zeros(1,NP)+1/NP;
#  end


#  function p=Gauss(x,u,sigma)
#  %ガウス分布の確率密度を計算する関数
#  p=1/sqrt(2*pi*sigma^2)*exp(-(x-u)^2/(2*sigma^2));

#  %Calc Observation from noise prameter
#  function [z, x, xd, u] = Observation(x, xd, u, RFID,MAX_RANGE)
#  global Qsigma;
#  global Rsigma;

#  x=f(x, u);% Ground Truth
#  u=u+sqrt(Qsigma)*randn(2,1);%add Process Noise
#  xd=f(xd, u);% Dead Reckoning
#  %Simulate Observation
#  z=[];
#  for iz=1:length(RFID(:,1))
#  d=norm(RFID(iz,:)-x(1:2)');
#  if d<MAX_RANGE %観測範囲内
#  z=[z;[d+sqrt(Rsigma)*randn(1,1) RFID(iz,:)]];
#  end

import numpy as np
import math
import matplotlib.pyplot as plt

# Estimation parameter of EKF
Q = np.diag([0.1, 0.1, math.radians(1.0), 1.0])**2
R = np.diag([1.0, math.radians(40.0)])**2

#  Simulation parameter
Qsim = np.diag([0.5, 0.5])**2
Rsim = np.diag([1.0, math.radians(30.0)])**2

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range

# Particle filter parameter
NP = 100  # Number of Particle
NTh = NP / 2.0  # Number of particle for re-sampling

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


def jacobF(x, u):
    # Jacobian of Motion Model
    yaw = x[2, 0]
    u1 = u[0, 0]
    jF = np.matrix([
        [1.0, 0.0, -DT * u1 * math.sin(yaw), DT * u1 * math.cos(yaw)],
        [0.0, 1.0, DT * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF


def jacobH(x):
    # Jacobian of Observation Model
    jH = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH


def pf_estimation(xEst, PEst, z, u):

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


def main():
    print(__file__ + " start!!")

    time = 0.0

    # RFID positions [x, y]
    RFID = np.array([[10.0, 0.0],
                     [10.0, 10.0],
                     [0.0, 15.0],
                     [-5.0, 20.0]])

    # State Vector [x y yaw v]'
    xEst = np.matrix(np.zeros((4, 1)))
    xTrue = np.matrix(np.zeros((4, 1)))
    PEst = np.eye(4)

    px = np.matrix(np.zeros((4, NP)))  # Particle store
    pw = np.matrix(np.zeros((1, NP))) + 1.0 / NP  # Particle weight
    #  print(px)
    #  print(pw)

    xDR = np.matrix(np.zeros((4, 1)))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((1, 2))

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u)

        xEst, PEst = pf_estimation(xEst, PEst, z, ud)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.vstack((hz, z))

        if show_animation:
            plt.cla()
            plt.plot(hz[:, 0], hz[:, 1], ".g")
            plt.plot(RFID[:, 0], RFID[:, 1], "*k")
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
