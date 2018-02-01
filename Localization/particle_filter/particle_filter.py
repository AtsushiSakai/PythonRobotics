"""

Particle Filter localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""

#  pw=Normalize(pw,NP);%正規化
#  [px,pw]=Resampling(px,pw,NTh,NP);%リサンプリング
#  xEst=px*pw';%最終推定値は期待値

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


def observation(xTrue, xd, u, RFID):

    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.matrix(np.zeros((0, 3)))

    for i in range(len(RFID[:, 0])):

        dx = xTrue[0, 0] - RFID[i, 0]
        dy = xTrue[1, 0] - RFID[i, 1]
        d = math.sqrt(dx**2 + dy**2)
        if d <= MAX_RANGE:
            zi = np.matrix([d, RFID[i, 0], RFID[i, 1]])
            z = np.vstack((z, zi))

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


def pf_estimation(px, pw, xEst, PEst, z, u):

    #  Predict
    for ip in range(NP):
        x = px[:, ip]
        #  w = pw[ip]

        ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
        ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1]
        ud = np.matrix([ud1, ud2]).T

        x = motion_model(x, ud)

        px[:, ip] = x

        #  Calc Inportance Weight
        for i in range(len(z[:, 0])):
            dx = x[0, 0] - z[i, 1]
            dy = x[1, 0] - z[i, 2]
            prez = math.sqrt(dx**2 + dy**2)
            dz = prez - z[i, 0]
            #  w=w*Gauss(dz,0,sqrt(R));
            #  end
            #  px(:,ip)=x;%格納
            #  pw(ip)=w;
            #  end

    xEst = px * pw.T

    return xEst, PEst, px, pw


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

    xDR = np.matrix(np.zeros((4, 1)))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        xEst, PEst, px, pw = pf_estimation(px, pw, xEst, PEst, z, ud)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        if show_animation:
            plt.cla()

            for i in range(len(z[:, 0])):
                plt.plot([xTrue[0, 0], z[i, 1]], [xTrue[1, 0], z[i, 2]], "-k")
            plt.plot(RFID[:, 0], RFID[:, 1], "*k")
            plt.plot(px[0, :], px[1, :], ".r")
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
