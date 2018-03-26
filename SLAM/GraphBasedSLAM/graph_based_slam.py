"""

Graph SLAM example

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import copy
import itertools
import matplotlib.pyplot as plt


#  Simulation parameter
Qsim = np.diag([0.2, math.radians(1.0)])**2
Rsim = np.diag([1.0, math.radians(10.0)])**2

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM srate size [x,y]

C_SIGMA1 = 1.0
C_SIGMA2 = 0.1
C_SIGMA3 = 0.1

MAX_ITR = 20

show_animation = True


class Edge():

    def __init__(self):
        self.e = np.zeros((3, 1))
        self.omega = np.zeros((3, 3))  # information matrix
        self.d_t = 0.0
        self.d_td = 0.0
        self.yaw_t = 0.0
        self.yaw_td = 0.0
        self.angle_t = 0.0
        self.angle_td = 0.0
        self.id1 = 0
        self.id2 = 0


def cal_observation_sigma(d):

    sigma = np.zeros((3, 3))
    sigma[0, 0] = (d * C_SIGMA1)**2
    sigma[1, 1] = (d * C_SIGMA2)**2
    sigma[2, 2] = C_SIGMA3**2

    return sigma


def calc_rotational_matrix(angle):

    Rt = np.matrix([[math.cos(angle), -math.sin(angle), 0],
                    [math.sin(angle), math.cos(angle), 0],
                    [0, 0, 1.0]])
    return Rt


def calc_edges(xlist, zlist):

    edges = []
    zids = list(itertools.combinations(range(len(zlist)), 2))

    for (t, td) in zids:
        xt, yt, yawt = xlist[0, t], xlist[1, t], xlist[2, t]
        xtd, ytd, yawtd = xlist[0, td], xlist[1, td], xlist[2, td]
        dt, anglet, phit = zlist[t][0, 0], zlist[t][1, 0], zlist[t][2, 0]
        dtd, angletd, phitd = zlist[td][0, 0], zlist[td][1, 0], zlist[td][2, 0]

        edge = Edge()

        tangle1 = yawt + anglet
        tangle2 = yawt + anglet
        t1 = dt * math.cos(tangle1)
        t2 = dtd * math.cos(tangle2)
        t3 = dt * math.sin(tangle1)
        t4 = dtd * math.sin(tangle2)

        edge.e[0, 0] = xtd - xt - t1 + t2
        edge.e[1, 0] = ytd - yt - t3 + t4
        edge.e[2, 0] = yawtd - yawt - phit + phitd

        sig_t = cal_observation_sigma(dt)
        sig_td = cal_observation_sigma(dtd)

        Rt = calc_rotational_matrix(tangle1)
        Rtd = calc_rotational_matrix(tangle2)

        edge.omega = np.linalg.inv(Rt * sig_t * Rt.T + Rtd * sig_td * Rtd.T)
        edge.d_t, edge.d_td = dt, dtd
        edge.yaw_t, edge.yaw_td = yawt, yawtd
        edge.angle_t, edge.angle_td = anglet, angletd
        edge.id1, edge.id2 = t, td

        edges.append(edge)

    return edges


def calc_jacobian(edge):
    t = edge.yaw_t + edge.angle_t
    A = np.matrix([[-1.0, 0, edge.d_t * math.sin(t)],
                   [0, -1.0, -edge.d_t * math.cos(t)],
                   [0, 0, -1.0]])

    td = edge.yaw_td + edge.angle_td
    B = np.matrix([[1.0, 0, -edge.d_td * math.sin(td)],
                   [0, 1.0, edge.d_td * math.cos(td)],
                   [0, 0, 1.0]])

    return A, B


def fill_H_and_b(H, b, edge):

    A, B = calc_jacobian(edge)

    id1 = edge.id1 * 3
    id2 = edge.id2 * 3

    H[id1:id1 + 3, id1:id1 + 3] += A.T * edge.omega * A
    H[id1:id1 + 3, id2:id2 + 3] += A.T * edge.omega * B
    H[id2:id2 + 3, id1:id1 + 3] += B.T * edge.omega * A
    H[id2:id2 + 3, id2:id2 + 3] += B.T * edge.omega * B

    b[id1:id1 + 3, 0] += (A.T * edge.omega * edge.e)
    b[id2:id2 + 3, 0] += (B.T * edge.omega * edge.e)

    return H, b


def graph_based_slam(xEst, PEst, u, z, hxDR, hz):

    x_opt = copy.deepcopy(hxDR)
    n = len(hz) * 3

    for itr in range(MAX_ITR):
        edges = calc_edges(x_opt, hz)
        #  print("n edges:", len(edges))

        H = np.matrix(np.zeros((n, n)))
        b = np.matrix(np.zeros((n, 1)))

        for edge in edges:
            H, b = fill_H_and_b(H, b, edge)

        H[0:3, 0:3] += np.identity(3) * 10000  # to fix origin

        dx = - np.linalg.inv(H).dot(b)

        for i in range(len(hz)):
            x_opt[0:3, i] += dx[i * 3:i * 3 + 3, 0]

        diff = dx.T.dot(dx)
        print("iteration: %d, diff: %f" % (itr + 1, diff))
        if diff < 1.0e-5:
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
    z = np.matrix(np.zeros((0, 4)))

    for i in range(len(RFID[:, 0])):

        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.sqrt(dx**2 + dy**2)
        angle = pi_2_pi(math.atan2(dy, dx)) - xTrue[2, 0]
        phi = angle + xTrue[2, 0]
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Qsim[0, 0]  # add noise
            anglen = angle + np.random.randn() * Qsim[1, 1]  # add noise
            zi = np.matrix([dn, anglen, phi, i])
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

    # RFID positions [x, y, yaw]
    RFID = np.array([[10.0, -2.0, 0.0],
                     [15.0, 10.0, 0.0],
                     [3.0, 15.0, 0.0],
                     [-5.0, 20.0, 0.0]])

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
