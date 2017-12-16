"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi)


bstacleR=0.5;%衝突判定用の障害物の半径

function dist=CalcDistEval(x,ob,R)
%障害物との距離評価値を計算する関数

dist=2;
for io=1:length(ob(:,1))
    disttmp=norm(ob(io,:)-x(1:2)')-R;%パスの位置と障害物とのノルム誤差を計算
    if dist>disttmp%最小値を見つける
        dist=disttmp;
    end
end

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
        self.max_accel = 0.5
        self.max_dyawrate = 40.0 * math.pi / 180.0
        self.v_reso = 0.01
        self.yawrate_reso = 0.1 * math.pi / 180.0

        self.dt = 0.1  # [s]
        self.predict_time = 2.0  # [s]
        self.to_goal_cost_gain = 1.0  # [s]
        self.speed_cost_gain = 1.0  # [s]
        self.robot_radius = 1.0  # [s]


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


def calc_dynamic_window(x, config):

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]
    #  print(Vs, Vd)

    #  [vmin,vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    #  print(dw)

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


def evaluate_path(x, dw, config, goal, ob):

    xinit = x[:]
    min_cost = 10000.0
    min_u = [0.0, 0.0]
    best_traj = None

    for v in np.arange(dw[0], dw[1], config.v_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):
            traj = calc_trajectory(xinit, v, y, config)
            #  print(v, ",", y)

            to_goal_cost = calc_to_goal_cost(traj, goal, config)
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - traj[-1, 3])
            ob_cost = calc_obstacle_cost(traj, ob, config)
            #  print(ob_cost)

            final_cost = to_goal_cost + speed_cost + ob_cost

            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, y]
                best_traj = traj

    if best_traj is not None:
        plt.plot(best_traj[:, 0], best_traj[:, 1])
    #  print(min_u)
    #  input()

    return min_u


def calc_obstacle_cost(traj, ob, config):

    for ii in range(len(traj[:, 1])):
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)
            if r <= config.robot_radius:
                return float("Inf")

    return 0.0


def calc_to_goal_cost(traj, goal, config):
    cost = 0

    #  traj_end_yaw = traj[-1, 2]
    dy = goal[0] - traj[-1, 0]
    dx = goal[1] - traj[-1, 1]

    #  goal_yaw = math.atan2(dy, dx)

    goal_dis = math.sqrt(dx**2 + dy**2)
    #  print(goal_dis)

    #  cost = config.angle_cost_gain * abs(goal_yaw - traj_end_yaw) + goal_dis
    cost = config.to_goal_cost_gain * goal_dis
    #  print(cost)

    return cost


def dwa_control(x, u, config, dt, goal, ob):

    dw = calc_dynamic_window(x, config)

    u = evaluate_path(x, dw, config, goal, ob)

    return u


def main():
    print(__file__ + " start!!")

    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
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
                    [8.0, 9.0],
                    [7.0, 9.0],
                    [12.0, 12.0]
                    ])

    dt = 0.1
    u = np.array([0.0, 0.0])
    config = Config()

    for i in range(1000):
        plt.cla()
        u = dwa_control(x, u, config, dt, goal, ob)

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
