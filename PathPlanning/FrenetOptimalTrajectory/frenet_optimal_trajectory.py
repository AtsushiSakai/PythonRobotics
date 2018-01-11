"""
Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import matplotlib.pyplot as plt
import copy


class quinic_polynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):

        # calc coefficient of quinic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[T**3, T**4, T**5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T**2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3

        return xt


class quartic_polynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, T):

        # calc coefficient of quinic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * T ** 2, 4 * T ** 3],
                      [6 * T, 12 * T ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2

        return xt


class Frenet_path:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.s = []
        self.s_d = []
        self.s_dd = []

        self.x = []
        self.y = []
        self.yaw = []
        self.c = []


max_speed = 50.0 / 3.6
max_accel = 2.0
max_curvature = 1.0
maxd = 5.0
dd = 1.0
dt = 1.0
T = 10.0
target_speed = 30.0 / 3.6
dv = 5.0 / 3.6
nv = 2


def calc_frenet_paths(c_speed, c_d):

    frenet_paths = []

    for di in np.arange(-maxd, maxd, dd):
        for Ti in np.arange(dt, T, dt):
            fp = Frenet_path()

            lat_qp = quinic_polynomial(c_d, 0.0, 0.0, di, 0.0, 0.0, Ti)

            for t in np.arange(0.0, Ti, 0.1):
                fp.t.append(t)
                fp.d.append(lat_qp.calc_point(t))
                fp.d_d.append(lat_qp.calc_first_derivative(t))
                fp.d_dd.append(lat_qp.calc_second_derivative(t))

            for tv in np.arange(target_speed - dv * nv, target_speed + dv * nv, dv):
                tfp = copy.deepcopy(fp)
                lon_qp = quartic_polynomial(
                    0.0, c_speed, 0.0, tv, 0.0, Ti)

                for t in fp.t:
                    tfp.s.append(lon_qp.calc_point(t))
                    tfp.s_d.append(lon_qp.calc_first_derivative(t))
                    tfp.s_dd.append(lon_qp.calc_second_derivative(t))

                frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist):

    for fp in fplist:
        fp.x = fp.s
        fp.y = fp.d

    return fplist


def check_paths(fplist):

    okind = []
    for i in range(len(fplist)):
        # speed check
        if any([v > max_speed for v in fplist[i].s_d]):
            continue
        elif any([abs(a) > max_accel for a in fplist[i].s_dd]):
            continue

        okind.append(i)

    return [fplist[i] for i in okind]


def frenet_optimal_planning():

    c_speed = 0.0
    c_d = 1.0

    fplist = calc_frenet_paths(c_speed, c_d)
    fplist = calc_global_paths(fplist)
    fplist = check_paths(fplist)

    for fp in fplist:
        plt.plot(fp.x, fp.y)


def main():
    print(__file__ + " start!!")

    frenet_optimal_planning()

    plt.axis("equal")
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
