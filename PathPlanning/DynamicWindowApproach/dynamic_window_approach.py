"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi)


bstacleR=0.5;%衝突判定用の障害物の半径
global dt; dt=0.1;%刻み時間[s]

%ロボットの力学モデル
%[最高速度[m/s],最高回頭速度[rad/s],最高加減速度[m/ss],最高加減回頭速度[rad/ss],
% 速度解像度[m/s],回頭速度解像度[rad/s]]
Kinematic=[1.0,toRadian(20.0),0.2,toRadian(50.0),0.01,toRadian(1)];

%評価関数のパラメータ [heading,dist,velocity,predictDT]
evalParam=[0.1,0.2,0.1,3.0];
area=[-1 11 -1 11];%シミュレーションエリアの広さ [xmin xmax ymin ymax]

%シミュレーション結果
result.x=[];
tic;
%movcount=0;
% Main loop
for i=1:5000
    %DWAによる入力値の計算
    [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);
    x=f(x,u);%運動モデルによる移動

    %シミュレーション結果の保存
    result.x=[result.x; x'];

    %ゴール判定
    if norm(x(1:2)-goal')<0.5
        disp('Arrive Goal!!');break;
    end

    %====Animation====
    hold off;
    ArrowLength=0.5;%矢印の長さ
    %ロボット
    quiver(x(1),x(2),ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'ok');hold on;
    plot(result.x(:,1),result.x(:,2),'-b');hold on;
    plot(goal(1),goal(2),'*r');hold on;
    plot(obstacle(:,1),obstacle(:,2),'*k');hold on;
    %探索軌跡表示
    if ~isempty(traj)
        for it=1:length(traj(:,1))/5
            ind=1+(it-1)*5;
            plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
        end
    end
    axis(area);
    grid on;
    drawnow;
    %movcount=movcount+1;
    %mov(movcount) = getframe(gcf);% アニメーションのフレームをゲットする
end
figure(2)
plot(result.x(:,4));
toc
%movie2avi(mov,'movie.avi');


function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)
%DWAによる入力値の計算をする関数

%Dynamic Window[vmin,vmax,ωmin,ωmax]の作成
Vr=CalcDynamicWindow(x,model);
%評価関数の計算
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end

%各評価関数の正規化
evalDB=NormalizeEval(evalDB);

%最終評価値の計算
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
end
evalDB=[evalDB feval];

[maxv,ind]=max(feval);%最も評価値が大きい入力値のインデックスを計算
u=evalDB(ind,1:2)';%評価値が高い入力値を返す

function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
%各パスに対して評価値を計算する関数
evalDB=[];
trajDB=[];

for vt=Vr(1):model(5):Vr(2)
    for ot=Vr(3):model(6):Vr(4)
        %軌跡の推定
        [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4),model);
        %各評価関数の計算
        heading=CalcHeadingEval(xt,goal);
        dist=CalcDistEval(xt,ob,R);
        vel=abs(vt);

        evalDB=[evalDB;[vt ot heading dist vel]];
        trajDB=[trajDB;traj];
    end
end

function EvalDB=NormalizeEval(EvalDB)
%評価値を正規化する関数
if sum(EvalDB(:,3))~=0
    EvalDB(:,3)=EvalDB(:,3)/sum(EvalDB(:,3));
end
if sum(EvalDB(:,4))~=0
    EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
end
if sum(EvalDB(:,5))~=0
    EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
end

function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model)
%軌跡データを作成する関数
global dt;
time=0;
u=[vt;ot];%入力値
traj=x;%軌跡データ
while time<=evaldt
    time=time+dt;%シミュレーション時間の更新
    x=f(x,u);%運動モデルによる推移
    traj=[traj x];
end

function stopDist=CalcBreakingDist(vel,model)
%現在の速度から力学モデルに従って制動距離を計算する関数
global dt;
stopDist=0;
while vel>0
    stopDist=stopDist+vel*dt;%制動距離の計算
    vel=vel-model(3)*dt;%最高原則
end

function dist=CalcDistEval(x,ob,R)
%障害物との距離評価値を計算する関数

dist=2;
for io=1:length(ob(:,1))
    disttmp=norm(ob(io,:)-x(1:2)')-R;%パスの位置と障害物とのノルム誤差を計算
    if dist>disttmp%最小値を見つける
        dist=disttmp;
    end
end

function heading=CalcHeadingEval(x,goal)
%headingの評価関数を計算する関数

theta=toDegree(x(3));%ロボットの方位
goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));%ゴールの方位

if goalTheta>theta
    targetTheta=goalTheta-theta;%ゴールまでの方位差分[deg]
else
    targetTheta=theta-goalTheta;%ゴールまでの方位差分[deg]
end

heading=180-targetTheta;

function Vr=CalcDynamicWindow(x,model)
%モデルと現在の状態からDyamicWindowを計算
global dt;
%車両モデルによるWindow
Vs=[0 model(1) -model(2) model(2)];

%運動モデルによるWindow
Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];

%最終的なDynamic Windowの計算
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
%[vmin,vmax,ωmin,ωmax]

function x = f(x, u)
% Motion Model
global dt;

F = [1 0 0 0 0
     0 1 0 0 0
     0 0 1 0 0
     0 0 0 0 0
     0 0 0 0 0];

B = [dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0
    0 1];

x= F*x+B*u;

function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;

function degree = toDegree(radian)
% radian to degree
degree = radian/pi*180;
"""

import math
import numpy as np
import matplotlib.pyplot as plt


class Config():

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0
        self.min_speed = -0.5
        self.max_yawrate = 40.0 * math.pi / 180.0
        self.max_accel = 0.2
        self.max_dyawrate = 40.0 * math.pi / 180.0
        self.v_reso = 0.1
        self.yawrate_reso = 0.1 * math.pi / 180.0

        self.dt = 0.1  # [s]
        self.predict_time = 5.0  # [s]


def plot_arrow(x, y, yaw, length=0.5, width=0.1):
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def motion(x, u, dt):

    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[2] += u[1] * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, rspec, dt):

    # Dynamic window from robot specification
    Vs = [rspec.min_speed, rspec.max_speed,
          -rspec.max_yawrate, rspec.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - rspec.max_accel * dt,
          x[3] + rspec.max_accel * dt,
          x[4] - rspec.max_dyawrate * dt,
          x[4] + rspec.max_dyawrate * dt]
    print(Vs, Vd)

    #  [vmin,vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    print(dw)

    return dw


def calc_trajectory(xinit, v, y, config):

    x = np.array(xinit)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt

    #  print(len(traj))
    return traj


def calc_cost(x, dw, rspec, dt):

    xinit = x[:]

    for v in np.arange(dw[0], dw[1], rspec.v_reso):
        for y in np.arange(dw[2], dw[3], rspec.yawrate_reso):
            traj = calc_trajectory(xinit, v, y, rspec)
            plt.plot(traj[:, 0], traj[:, 1])


def dwa_control(x, u, rspec, dt):

    dw = calc_dynamic_window(x, rspec, dt)

    calc_cost(x, dw, rspec, dt)

    u = np.array([0.5, 0.0])

    return u


def main():
    print(__file__ + " start!!")

    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi / 4.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([10, 10])
    # obstacles [x(m) y(m), ....]
    ob = np.matrix([[-1, -1],
                    [0, 2],
                    [4.0, 2.0],
                    [5.0, 4.0],
                    [5.0, 5.0],
                    [5.0, 6.0],
                    [5.0, 9.0],
                    [8.0, 8.0],
                    [8.0, 9.0],
                    [7.0, 9.0],
                    [12.0, 12.0]
                    ])

    dt = 0.1
    u = np.array([0.0, 0.0])
    rspec = Config()

    for i in range(100):
        plt.cla()
        u = dwa_control(x, u, rspec, dt)

        x = motion(x, u, dt)

        plt.plot(x[0], x[1], "xr")
        plt.plot(goal[0], goal[1], "xb")
        plt.plot(ob[:, 0], ob[:, 1], "ok")
        plot_arrow(x[0], x[1], x[2])
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)

    print("Done")
    plt.show()


if __name__ == '__main__':
    main()
