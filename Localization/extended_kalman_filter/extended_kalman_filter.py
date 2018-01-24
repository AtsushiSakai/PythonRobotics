"""

Extended kalman filter (EKF) localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import matplotlib.pyplot as plt

# Estimation parameter
Q = np.diag([0.5, 0.5])**2
R = np.diag([1.0, math.radians(30.0)])**2

#  Simulation parameter
Qsim = np.diag([0.5, 0.5])**2
Rsim = np.diag([1.0, math.radians(30.0)])**2


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


#  function x = f(x, u)
#  % Motion Model
#  global dt;

#  F = [1 0 0 0
#  0 1 0 0
#  0 0 1 0
#  0 0 0 0];

#  B = [
#  dt*cos(x(3)) 0
#  dt*sin(x(3)) 0
#  0 dt
#  1 0];

#  x= F*x+B*u;

#  function jF = jacobF(x, u)
#  % Jacobian of Motion Model
#  global dt;

#  jF=[
#  1 0 0 0
#  0 1 0 0
#  -dt*u(1)*sin(x(3)) dt*u(1)*cos(x(3)) 1 0
#  dt*cos(x(3)) dt*sin(x(3)) 0 1];

#  function z = h(x)
#  %Observation Model

#  H = [1 0 0 0
#  0 1 0 0
#  0 0 1 0
#  0 0 0 1 ];

#  z=H*x;

#  function jH = jacobH(x)
#  %Jacobian of Observation Model

#  jH =[1 0 0 0
#  0 1 0 0
#  0 0 1 0
#  0 0 0 1];


#  function angle=Pi2Pi(angle)
#  %ロボットの角度を-pi~piの範囲に補正する関数
#  angle = mod(angle, 2*pi);

#  i = find(angle>pi);
#  angle(i) = angle(i) - 2*pi;

#  i = find(angle<-pi);
#  angle(i) = angle(i) + 2*pi;


DT = 0.1  # time tick [s]
SIM_TIME = 60.0  # simulation time [s]


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


def ekf_estimation(xEst, PEst, u):

    #  Predict
    xPred = motion_model(xEst, u)
    #  F=jacobF(xPred, u);
    #  PPred= F*PEst*F' + Q;

    #  Update
    #  H=jacobH(xPred);
    #  y = z - h(xPred);
    #  S = H*PPred*H' + R;
    #  K = PPred*H'*inv(S);
    #  xEst = xPred + K*y;
    xEst = xPred
    #  PEst = (eye(size(xEst,1)) - K*H)*PPred;

    return xEst


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

        xEst = ekf_estimation(xEst, PEst, ud)

        plt.plot(xTrue[0, 0], xTrue[1, 0], ".b")
        plt.plot(xDR[0, 0], xDR[1, 0], ".k")
        plt.plot(z[0, 0], z[0, 1], ".g")
        plt.plot(xEst[0, 0], xEst[1, 0], ".r")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)


if __name__ == '__main__':
    main()
