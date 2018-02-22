"""

Iterative Closet Point (ICP) SLAM example

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
import matplotlib.pyplot as plt

#  % 点をランダムでばら撒く(t-1の時の点群)
#  data1=fieldLength*rand(2,nPoint)-fieldLength/2;

#  % data2= data1を移動させる & ノイズ付加
#  % 回転方向 ＆ ノイズ付加
#  theta=toRadian(motion(3))+toRadian(thetaSigma)*rand(1);
#  % 並進ベクトル ＆ ノイズ付加
#  t=repmat(motion(1:2)',1,nPoint)+transitionSigma*randn(2,nPoint);
#  % 回転行列の作成
#  A=[cos(theta) sin(theta);-sin(theta) cos(theta)];
#  % data1を移動させてdata2を作る
#  data2=t+A*data1;

#  function [R, t]=ICPMatching(data1, data2)
#  % ICPアルゴリズムによる、並進ベクトルと回転行列の計算を実施する関数
#  % data1 = [x(t)1 x(t)2 x(t)3 ...]
#  % data2 = [x(t+1)1 x(t+1)2 x(t+1)3 ...]
#  % x=[x y z]'


#  while ~(dError < EPS)
#  count=count+1;

#  [ii, error]=FindNearestPoint(data1, data2);%最近傍点探索
#  [R1, t1]=SVDMotionEstimation(data1, data2, ii);%特異値分解による移動量推定
#  %計算したRとtで点群とRとtの値を更新
#  data2=R1*data2;
#  data2=[data2(1,:)+t1(1) ; data2(2,:)+t1(2)];
#  R = R1*R;
#  t = R1*t + t1;

#  dError=abs(preError-error);%エラーの改善量
#  preError=error;%一つ前のエラーの総和値を保存

#  if count > maxIter %収束しなかった
#  disp('Max Iteration');return;
#  end
#  end
#  disp(['Convergence:',num2str(count)]);

#  function [index, error]=FindNearestPoint(data1, data2)
#  %data2に対するdata1の最近傍点のインデックスを計算する関数
#  m1=size(data1,2);
#  m2=size(data2,2);
#  index=[];
#  error=0;

#  for i=1:m1
#  dx=(data2-repmat(data1(:,i),1,m2));
#  dist=sqrt(dx(1,:).^2+dx(2,:).^2);
#  [dist, ii]=min(dist);
#  index=[index; ii];
#  error=error+dist;
#  end

#  function [R, t]=SVDMotionEstimation(data1, data2, index)
#  %特異値分解法による並進ベクトルと、回転行列の計算

#  %各点群の重心の計算
#  M = data1;
#  mm = mean(M,2);
#  S = data2(:,index);
#  ms = mean(S,2);

#  %各点群を重心中心の座標系に変換
#  Sshifted = [S(1,:)-ms(1); S(2,:)-ms(2);];
#  Mshifted = [M(1,:)-mm(1); M(2,:)-mm(2);];

#  W = Sshifted*Mshifted';
#  [U,A,V] = svd(W);%特異値分解

#  R = (U*V')';%回転行列の計算
#  t = mm - R*ms;%並進ベクトルの計算


def ICP_matching(pdata, data):
    R = np.eye(2)  # rotation matrix
    T = np.zeros((2, 1))  # translation vector

    #  ICP
    EPS = 0.0001
    maxIter = 100

    dError = 1000.0
    preError = 1000.0
    count = 0

    while dError >= EPS:
        count += 1

        error = nearest_neighbor_assosiation(pdata, data)
        Rt, Tt = SVD_motion_estimation(pdata, data)
        print(error)

        data = (Rt * data) + Tt
        R = R * Rt
        T = R * T + Tt

        dError = abs(preError - error)
        preError = error

        if dError <= EPS:
            print("Converge", dError)
            plt.plot(data[0, :], data[1, :], "*k")
            break
        elif maxIter <= count:
            break

    return R, T


def nearest_neighbor_assosiation(pdata, data):

    ddata = pdata - data
    #  print(ddata)

    d = np.linalg.norm(ddata, axis=0)

    error = sum(d)

    return error


def SVD_motion_estimation(pdata, data):

    pm = np.matrix(np.mean(pdata, axis=1))
    cm = np.matrix(np.mean(data, axis=1))

    pshift = np.matrix(pdata - pm)
    cshift = np.matrix(data - cm)

    W = cshift * pshift.T
    u, s, vh = np.linalg.svd(W)

    R = (u * vh).T
    t = pm - R * cm

    return R, t


def main():
    print(__file__ + " start!!")

    # simulation parameters
    nPoint = 10
    fieldLength = 50.0
    motion = [0.5, 1.0, math.radians(-40.0)]  # movement [x[m],y[m],yaw[deg]]
    #  transitionSigma=0.01;%並進方向の移動誤差標準偏差[m]
    #  thetaSigma=1;   %回転方向の誤差標準偏差[deg]

    # previous point data
    px = (np.random.rand(nPoint) - 0.5) * fieldLength
    py = (np.random.rand(nPoint) - 0.5) * fieldLength

    cx = [math.cos(motion[2]) * x - math.sin(motion[2]) * y + motion[0]
          for (x, y) in zip(px, py)]
    cy = [math.sin(motion[2]) * x + math.cos(motion[2]) * y + motion[1]
          for (x, y) in zip(px, py)]

    pdata = np.matrix(np.vstack((px, py)))
    #  print(pdata)
    data = np.matrix(np.vstack((cx, cy)))
    #  print(data)

    R, T = ICP_matching(pdata, data)

    fdata = (R * data) + T

    plt.plot(px, py, ".b")
    plt.plot(cx, cy, ".r")
    plt.plot(fdata[0, :], fdata[1, :], "xk")
    plt.plot(0.0, 0.0, "xr")
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
