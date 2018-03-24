"""

Graph SLAM example

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import copy
import matplotlib.pyplot as plt


# EKF state covariance
Cx = np.diag([0.5, 0.5, math.radians(30.0)])**2

#  Simulation parameter
Qsim = np.diag([0.2, math.radians(1.0)])**2
Rsim = np.diag([1.0, math.radians(10.0)])**2

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM srate size [x,y]

MAX_ITR = 20

show_animation = True


def graph_based_slam(xEst, PEst, u, z, hxDR, hz):

    x_opt = copy.deepcopy(hxDR)

    for itr in range(20):
        #  pos_edges = []

        #  # このfor文では、HalfEdgeからグラフの辺を作っていきます。
        #  for i in range(len(actual_landmarks.positions)):                           # ランドマークごとにHalfEdgeからEdgeを作る
        #  es = list(filter(lambda e: e.landmark_id == i, obs_edges))      # 同じランドマークIDを持つHalfEdgeの抽出
        #  ps = list(itertools.combinations(es,2))                                       # esの要素のペアを全通り作る
        #  for p in ps:
            #  pos_edges.append(Edge(p[0],p[1]))                                    # エッジを登録

        n = len(hz) * 3
        H = np.zeros((n, n))
        b = np.zeros((n, 1))

        #  for e in pos_edges:
        #  e.addInfo(matH,vecb)

        #  H[0:3, 0:3] += np.identity(3) * 10000  # to fix origin
        H += np.identity(n) * 10000  # to fix origin

        dx = - np.linalg.inv(H).dot(b)
        #  print(dx)

        for i in range(len(hz)):
            x_opt[0, i] += dx[i * 3, 0]
            x_opt[1, i] += dx[i * 3 + 1, 0]
            x_opt[2, i] += dx[i * 3 + 2, 0]

        #  # HalfEdgeに登録してある推定値も更新
        #  for e in obs_edges:
        #  e.update(robot.guess_poses)

        diff = dx.T.dot(dx)
        print("iteration: %d, diff: %f" % (itr + 1, diff))
        if dx[0, 0] < 1.0e-5:
            break

    return x_opt, None


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

        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.sqrt(dx**2 + dy**2)
        angle = pi_2_pi(math.atan2(dy, dx))
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Qsim[0, 0]  # add noise
            anglen = angle + np.random.randn() * Qsim[1, 1]  # add noise
            zi = np.matrix([dn, anglen, i])
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


def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def main():
    print(__file__ + " start!!")

    time = 0.0

    # RFID positions [x, y]
    RFID = np.array([[10.0, -2.0],
                     [15.0, 10.0],
                     [3.0, 15.0],
                     [-5.0, 20.0]])

    # State Vector [x y yaw v]'
    xEst = np.matrix(np.zeros((STATE_SIZE, 1)))
    xTrue = np.matrix(np.zeros((STATE_SIZE, 1)))
    PEst = np.eye(STATE_SIZE)

    xDR = np.matrix(np.zeros((STATE_SIZE, 1)))  # Dead reckoning

    # history
    hxTrue = xTrue
    hxDR = xTrue
    hz = []

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        hxDR = np.hstack((hxDR, xDR))
        hz.append(z)

        x_opt, PEst = graph_based_slam(xEst, PEst, ud, z, hxDR, hz)

        # store data history
        hxTrue = np.hstack((hxTrue, xTrue))

        if show_animation:
            plt.cla()

            plt.plot(RFID[:, 0], RFID[:, 1], "*k")
            plt.plot(xEst[0], xEst[1], ".r")

            plt.plot(np.array(hxTrue[0, :]).flatten(),
                     np.array(hxTrue[1, :]).flatten(), "-b")
            plt.plot(np.array(hxDR[0, :]).flatten(),
                     np.array(hxDR[1, :]).flatten(), "-k")
            plt.plot(np.array(x_opt[0, :]).flatten(),
                     np.array(x_opt[1, :]).flatten(), "-r")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
