"""
Mix Integer Optimization based path planner

author: Atsushi Sakai
"""


import cvxpy
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotrecorder import matplotrecorder
matplotrecorder.donothing = True


# parameter
A = np.matrix([[1.0, 0.0],
               [0.0, 1.0]])
B = np.matrix([[1.0, 1.0],
               [0.0, 1.0]])
q = np.matrix([[1.0],
               [1.0]])
r = np.matrix([[0.1],
               [0.1]])

u_max = 0.1  # input constraint
T = 30  # horizon length
M = 10000.0  # weight parameter of obstacle avoidanse


def control(s1, gs, ob):

    w = cvxpy.Variable(2, T)
    v = cvxpy.Variable(2, T)
    s = cvxpy.Variable(2, T)
    u = cvxpy.Variable(2, T)
    nob = len(ob)
    o = cvxpy.Bool(4 * nob, T)

    constraints = [cvxpy.abs(u) <= u_max]
    constraints.append(s[:, 0] == s1)

    obj = []
    for t in range(T):
        constraints.append(s[:, t] - gs <= w[:, t])
        constraints.append(-s[:, t] + gs <= w[:, t])
        constraints.append(u[:, t] <= v[:, t])
        constraints.append(-u[:, t] <= v[:, t])

        obj.append(t * q.T * w[:, t] + r.T * v[:, t])

        # obstable avoidanse
        for io in range(nob):
            ind = io * 4
            constraints.append(sum(o[ind:ind + 4, t]) <= 3)
            constraints.append(s[0, t] <= ob[io, 0] + M * o[ind + 0, t])
            constraints.append(-s[0, t] <= -ob[io, 1] + M * o[ind + 1, t])
            constraints.append(s[1, t] <= ob[io, 2] + M * o[ind + 2, t])
            constraints.append(-s[1, t] <= -ob[io, 3] + M * o[ind + 3, t])

    for t in range(T - 1):
        constraints.append(s[:, t + 1] == A * s[:, t] + B * u[:, t])

    objective = cvxpy.Minimize(sum(obj))

    prob = cvxpy.Problem(objective, constraints)

    prob.solve(solver=cvxpy.GUROBI)

    s_p = s.value
    u_p = u.value
    print("status:" + prob.status)

    return s_p, u_p


def plot_obstacle(ob):
    for i in range(len(ob)):
        x = [ob[i, 0], ob[i, 1], ob[i, 1], ob[i, 0], ob[i, 0]]
        y = [ob[i, 2], ob[i, 2], ob[i, 3], ob[i, 3], ob[i, 2]]
        plt.plot(x, y, "-g")


def main():
    print(__file__ + " start!!")

    s = np.matrix([10.0, 5.0]).T  # init state
    gs = np.matrix([5.0, 7.0]).T  # goal state

    ob = np.matrix([[7.0, 8.0, 3.0, 8.0],
                    [5.5, 6.0, 6.0, 10.0]])  # [xmin xmax ymin ymax]
    #  ob = np.matrix([[7.0, 8.0, 3.0, 8.0]])

    h_sx = []
    h_sy = []

    for i in range(10000):
        print("time:", i)
        s_p, u_p = control(s, gs, ob)

        s = A * s + B * u_p[:, 0]  # simulation

        if(math.sqrt((gs[0] - s[0]) ** 2 + (gs[1] - s[1]) ** 2) <= 0.1):
            print("Goal!!!")
            break

        h_sx.append(s[0, 0])
        h_sy.append(s[1, 0])

        plt.cla()
        plt.plot(gs[0], gs[1], "*r")
        plot_obstacle(ob)
        plt.plot(s_p[0, :], s_p[1, :], "xb")
        plt.plot(h_sx, h_sy, "-b")
        plt.plot(s[0], s[1], "or")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)
        matplotrecorder.save_frame()

    matplotrecorder.save_movie("animation.gif", 0.1)


if __name__ == '__main__':
    main()
