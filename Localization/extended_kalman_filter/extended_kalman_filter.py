"""

Extended kalman filter (EKF) localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import matplotlib.pyplot as plt


#  Covariance Matrix for motion
#  Q=diag([0.1 0.1 toRadian(1) 0.05]).^2;

#  % Covariance Matrix for observation
#  R=diag([1.5 1.5 toRadian(3) 0.05]).^2;

#  % Simulation parameter
#  global Qsigma
#  Qsigma=diag([0.1 toRadian(20)]).^2; %[v yawrate]

#  global Rsigma
#  Rsigma=diag([1.5 1.5 toRadian(3) 0.05]).^2;%[x y z yaw v]

#  PEst = eye(4);

#  u=doControl(time);
#  % Observation
#  [z,xTrue,xd,u]=Observation(xTrue, xd, u);

#  % ------ Kalman Filter --------
#  % Predict
#  xPred = f(xEst, u);
#  F=jacobF(xPred, u);
#  PPred= F*PEst*F' + Q;

#  % Update
#  H=jacobH(xPred);
#  y = z - h(xPred);
#  S = H*PPred*H' + R;
#  K = PPred*H'*inv(S);
#  xEst = xPred + K*y;
#  PEst = (eye(size(xEst,1)) - K*H)*PPred;

#  % Simulation Result
#  result.time=[result.time; time];
#  result.xTrue=[result.xTrue; xTrue'];
#  result.xd=[result.xd; xd'];
#  result.xEst=[result.xEst;xEst'];
#  result.z=[result.z; z'];
#  result.PEst=[result.PEst; diag(PEst)'];
#  result.u=[result.u; u'];

#  %Animation (remove some flames)
#  if rem(i,5)==0
#  %hold off;
#  plot(result.xTrue(:,1),result.xTrue(:,2),'.b');hold on;
#  plot(result.z(:,1),result.z(:,2),'.g');hold on;
#  plot(result.xd(:,1),result.xd(:,2),'.k');hold on;
#  plot(result.xEst(:,1),result.xEst(:,2),'.r');hold on;
#  ShowErrorEllipse(xEst,PEst);
#  axis equal;
#  grid on;
#  drawnow;
#  %movcount=movcount+1;
#  %mov(movcount) = getframe(gcf);% アニメーションのフレームをゲットする
#  end
#  end
#  toc
#  %アニメーション保存
#  %movie2avi(mov,'movie.avi');

#  DrawGraph(result);

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


#  function [z, x, xd, u] = Observation(x, xd, u)
#  %Calc Observation from noise prameter
#  global Qsigma;
#  global Rsigma;

#  x=f(x, u);% Ground Truth
#  u=u+Qsigma*randn(2,1);%add Process Noise
#  xd=f(xd, u);% Dead Reckoning
#  z=h(x+Rsigma*randn(4,1));%Simulate Observation


#  function []=DrawGraph(result)
#  %Plot Result

#  figure(1);
#  x=[ result.xTrue(:,1:2) result.xEst(:,1:2) result.z(:,1:2)];
#  set(gca, 'fontsize', 16, 'fontname', 'times');
#  plot(x(:,5), x(:,6),'.g','linewidth', 4); hold on;
#  plot(x(:,1), x(:,2),'-.b','linewidth', 4); hold on;
#  plot(x(:,3), x(:,4),'r','linewidth', 4); hold on;
#  plot(result.xd(:,1), result.xd(:,2),'--k','linewidth', 4); hold on;

#  title('EKF Localization Result', 'fontsize', 16, 'fontname', 'times');
#  xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
#  ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
#  legend('Ground Truth','GPS','Dead Reckoning','EKF','Error Ellipse');
#  grid on;
#  axis equal;

#  function angle=Pi2Pi(angle)
#  %ロボットの角度を-pi~piの範囲に補正する関数
#  angle = mod(angle, 2*pi);

#  i = find(angle>pi);
#  angle(i) = angle(i) - 2*pi;

#  i = find(angle<-pi);
#  angle(i) = angle(i) + 2*pi;


#  function radian = toRadian(degree)
#  % degree to radian
#  radian = degree/180*pi;

#  function degree = toDegree(radian)
#  % radian to degree
#  degree = radian/pi*180;

DT = 0.1  # time tick [s]
SIM_TIME = 60.0  # simulation time [s]


def do_control():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]

    u = np.matrix([v, yawrate]).T

    return u


#  z, xTrue, xd, u = Observation(xTrue, xd, u)
def observation(xTrue, xd, u):

    xTrue = motion_model(xTrue, u)

    return xTrue


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


def main():
    print(__file__ + " start!!")

    time = 0.0
    # State Vector [x y yaw v]'
    #  xEst = np.matrix(np.zeros((3, 1)))
    xTrue = np.matrix(np.zeros((4, 1)))

    # Dead Reckoning
    xDR = np.matrix(np.zeros((4, 1)))

    # Observation vector
    #  z = np.matrix(np.zeros((2, 1)))

    while SIM_TIME >= time:
        #  print(time)
        time += DT

        u = do_control()

        xTrue = observation(xTrue, xDR, u)

        plt.plot(xTrue[0, 0], xTrue[1, 0], ".r")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)


if __name__ == '__main__':
    main()
