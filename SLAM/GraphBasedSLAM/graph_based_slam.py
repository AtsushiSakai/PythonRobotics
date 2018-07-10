"""

Graph based SLAM example

author: Atsushi Sakai (@Atsushi_twi)

Ref

[A Tutorial on Graph-Based SLAM](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf)

"""

import numpy as np
import math
import copy
import itertools
import matplotlib.pyplot as plt


#  Simulation parameter
Qsim = np.diag([0.2, math.radians(1.0)])**2
Rsim = np.diag([0.1, math.radians(10.0)])**2

DT = 2.0  # time tick [s]
SIM_TIME = 100.0  # simulation time [s]
MAX_RANGE = 30.0  # maximum observation range
STATE_SIZE = 3  # State size [x,y,yaw]

# Covariance parameter of Graph Based SLAM
C_SIGMA1 = 0.1
C_SIGMA2 = 0.1
C_SIGMA3 = math.radians(1.0)

MAX_ITR = 20  # Maximum iteration

show_graph_dtime = 20.0  # [s]
show_animation = True


class Edge():

    def __init__(self):
        self.e = np.zeros((3, 1))
        self.omega = np.zeros((3, 3))  # information matrix
        self.d1 = 0.0
        self.d2 = 0.0
        self.yaw1 = 0.0
        self.yaw2 = 0.0
        self.angle1 = 0.0
        self.angle2 = 0.0
        self.id1 = 0
        self.id2 = 0


def cal_observation_sigma(d):

    sigma = np.zeros((3, 3))
    sigma[0, 0] = C_SIGMA1**2
    sigma[1, 1] = C_SIGMA2**2
    sigma[2, 2] = C_SIGMA3**2

    return sigma


def calc_rotational_matrix(angle):

    Rt = np.matrix([[math.cos(angle), -math.sin(angle), 0],
                    [math.sin(angle), math.cos(angle), 0],
                    [0, 0, 1.0]])
    return Rt


def calc_edge(x1, y1, yaw1, x2, y2, yaw2, d1,
              angle1, phi1, d2, angle2, phi2, t1, t2):
    edge = Edge()

    tangle1 = pi_2_pi(yaw1 + angle1)
    tangle2 = pi_2_pi(yaw2 + angle2)
    tmp1 = d1 * math.cos(tangle1)
    tmp2 = d2 * math.cos(tangle2)
    tmp3 = d1 * math.sin(tangle1)
    tmp4 = d2 * math.sin(tangle2)

    edge.e[0, 0] = x2 - x1 - tmp1 + tmp2
    edge.e[1, 0] = y2 - y1 - tmp3 + tmp4
    hyaw = phi1 - phi2 + angle1 - angle2
    edge.e[2, 0] = pi_2_pi(yaw2 - yaw1 - hyaw)

    Rt1 = calc_rotational_matrix(tangle1)
    Rt2 = calc_rotational_matrix(tangle2)

    sig1 = cal_observation_sigma(d1)
    sig2 = cal_observation_sigma(d2)

    edge.omega = np.linalg.inv(Rt1 * sig1 * Rt1.T + Rt2 * sig2 * Rt2.T)

    edge.d1, edge.d2 = d1, d2
    edge.yaw1, edge.yaw2 = yaw1, yaw2
    edge.angle1, edge.angle2 = angle1, angle2
    edge.id1, edge.id2 = t1, t2

    return edge


def calc_edges(xlist, zlist):

    edges = []
    cost = 0.0
    zids = list(itertools.combinations(range(len(zlist)), 2))

    for (t1, t2) in zids:
        x1, y1, yaw1 = xlist[0, t1], xlist[1, t1], xlist[2, t1]
        x2, y2, yaw2 = xlist[0, t2], xlist[1, t2], xlist[2, t2]

        if zlist[t1] is None or zlist[t2] is None:
            continue  # No observation

        for iz1 in range(len(zlist[t1][:, 0])):
            for iz2 in range(len(zlist[t2][:, 0])):
                if zlist[t1][iz1, 3] == zlist[t2][iz2, 3]:
                    d1 = zlist[t1][iz1, 0]
                    angle1, phi1 = zlist[t1][iz1, 1], zlist[t1][iz1, 2]
                    d2 = zlist[t2][iz2, 0]
                    angle2, phi2 = zlist[t2][iz2, 1], zlist[t2][iz2, 2]

                    edge = calc_edge(x1, y1, yaw1, x2, y2, yaw2, d1,
                                     angle1, phi1, d2, angle2, phi2, t1, t2)

                    edges.append(edge)
                    cost += (edge.e.T * edge.omega * edge.e)[0, 0]

    print("cost:", cost, ",nedge:", len(edges))
    return edges


def calc_jacobian(edge):
    t1 = edge.yaw1 + edge.angle1
    A = np.matrix([[-1.0, 0, edge.d1 * math.sin(t1)],
                   [0, -1.0, -edge.d1 * math.cos(t1)],
                   [0, 0, -1.0]])

    t2 = edge.yaw2 + edge.angle2
    B = np.matrix([[1.0, 0, -edge.d2 * math.sin(t2)],
                   [0, 1.0, edge.d2 * math.cos(t2)],
                   [0, 0, 1.0]])

    return A, B


def fill_H_and_b(H, b, edge):

    A, B = calc_jacobian(edge)

    id1 = edge.id1 * STATE_SIZE
    id2 = edge.id2 * STATE_SIZE

    H[id1:id1 + STATE_SIZE, id1:id1 + STATE_SIZE] += A.T * edge.omega * A
    H[id1:id1 + STATE_SIZE, id2:id2 + STATE_SIZE] += A.T * edge.omega * B
    H[id2:id2 + STATE_SIZE, id1:id1 + STATE_SIZE] += B.T * edge.omega * A
    H[id2:id2 + STATE_SIZE, id2:id2 + STATE_SIZE] += B.T * edge.omega * B

    b[id1:id1 + STATE_SIZE, 0] += (A.T * edge.omega * edge.e)
    b[id2:id2 + STATE_SIZE, 0] += (B.T * edge.omega * edge.e)

    return H, b


def graph_based_slam(x_init, hz):
    print("start graph based slam")

    zlist = copy.deepcopy(hz)
    zlist.insert(1, zlist[0])

    x_opt = copy.deepcopy(x_init)
    nt = x_opt.shape[1]
    n = nt * STATE_SIZE

    for itr in range(MAX_ITR):
        edges = calc_edges(x_opt, zlist)

        H = np.matrix(np.zeros((n, n)))
        b = np.matrix(np.zeros((n, 1)))

        for edge in edges:
            H, b = fill_H_and_b(H, b, edge)

        # to fix origin
        H[0:STATE_SIZE, 0:STATE_SIZE] += np.identity(STATE_SIZE)

        dx = - np.linalg.inv(H).dot(b)

        for i in range(nt):
            x_opt[0:3, i] += dx[i * 3:i * 3 + 3, 0]

        diff = dx.T.dot(dx)
        print("iteration: %d, diff: %f" % (itr + 1, diff))
        if diff < 1.0e-5:
            break

    return x_opt


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
        phi = pi_2_pi(math.atan2(dy, dx))
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
    return (angle + math.pi) % (2*math.pi) - math.pi


def main():
    print(__file__ + " start!!")

    time = 0.0

    # RFID positions [x, y, yaw]
    RFID = np.array([[10.0, -2.0, 0.0],
                     [15.0, 10.0, 0.0],
                     [3.0, 15.0, 0.0],
                     [-5.0, 20.0, 0.0],
                     [-5.0, 5.0, 0.0]
                     ])

    # State Vector [x y yaw v]'
    xTrue = np.matrix(np.zeros((STATE_SIZE, 1)))
    xDR = np.matrix(np.zeros((STATE_SIZE, 1)))  # Dead reckoning

    # history
    hxTrue = xTrue
    hxDR = xTrue
    hz = []
    dtime = 0.0

    while SIM_TIME >= time:
        time += DT
        dtime += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz.append(z)

        if dtime >= show_graph_dtime:
            x_opt = graph_based_slam(hxDR, hz)
            dtime = 0.0

            if show_animation:
                plt.cla()

                plt.plot(RFID[:, 0], RFID[:, 1], "*k")

                plt.plot(np.array(hxTrue[0, :]).flatten(),
                         np.array(hxTrue[1, :]).flatten(), "-b")
                plt.plot(np.array(hxDR[0, :]).flatten(),
                         np.array(hxDR[1, :]).flatten(), "-k")
                plt.plot(np.array(x_opt[0, :]).flatten(),
                         np.array(x_opt[1, :]).flatten(), "-r")
                plt.axis("equal")
                plt.grid(True)
                plt.title("Time" + str(time)[0:5])
                plt.pause(1.0)


if __name__ == '__main__':
    main()
