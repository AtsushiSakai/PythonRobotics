"""

Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import cubic_spline_planner


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
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


max_speed = 50.0 / 3.6
max_accel = 2.0
max_curvature = 1.0
maxd = 5.0
dd = 1.0
dt = 1.0
T = 10.0
target_speed = 30.0 / 3.6
robot_radius = 2.0
dv = 5.0 / 3.6
nv = 2

kj = 1.0
kt = 0.1
kd = 1.0
klat = 1.0
klon = 1.0


def calc_frenet_paths(c_speed, c_d, s0):

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
                    s0, c_speed, 0.0, tv, 0.0, Ti)

                for t in fp.t:
                    tfp.s.append(lon_qp.calc_point(t))
                    tfp.s_d.append(lon_qp.calc_first_derivative(t))
                    tfp.s_dd.append(lon_qp.calc_second_derivative(t))

                tfp.cd = kj * sum(tfp.d_dd) + kt * Ti + kd * tfp.d[-1]**2
                tfp.cv = kj * sum(tfp.s_dd) + kt * Ti + kd * \
                    (target_speed - tfp.s_d[-1])**2
                tfp.cf = klat * tfp.cd + klon * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):

    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            iyaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(iyaw + math.pi / 2.0)
            fy = iy + di * math.sin(iyaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.sqrt(dx**2 + dy**2))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob):

    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0])**2 + (iy - ob[i, 1])**2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= robot_radius**2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):

    okind = []
    for i in range(len(fplist)):
        if any([v > max_speed for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(a) > max_accel for a in fplist[i].s_dd]):  # Max accel check
            continue
        elif any([abs(c) > max_curvature for c in fplist[i].c]):  # Max curvature check
            continue
        elif not check_collision(fplist[i], ob):
            continue

        okind.append(i)

    return [fplist[i] for i in okind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, ob):

    fplist = calc_frenet_paths(c_speed, c_d, s0)
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist, ob)

    # find minimum cost path
    mincost = float("inf")
    bestpath = None
    for fp in fplist:
        #  plt.plot(fp.x, fp.y)
        if mincost >= fp.cf:
            mincost = fp.cf
            bestpath = fp

    return bestpath


def main():
    print(__file__ + " start!!")

    x = [0.0, 10.0, 20.5, 35.0, 70.5]
    y = [0.0, -6.0, 5.0, 6.5, 0.0]
    ob = np.array([[20.0, 10.0],
                   [30.0, 5.0]])

    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    c_speed = 10.0 / 3.6  # m/s
    c_d = 5.0  # [m]
    s0 = 0.0

    print(s0, c_d, c_speed)

    for i in range(100):
        plt.cla()
        plt.plot(rx, ry)
        plt.plot(ob[:, 0], ob[:, 1], "xk")

        path = frenet_optimal_planning(csp, s0, c_speed, c_d, ob)

        s0 = path.s[1]
        c_d = path.d[1]
        c_speed = path.s_d[1]
        print(s0, c_d, c_speed)

        plt.plot(path.x, path.y, "-or")

        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)
        #  input()

    plt.show()


if __name__ == '__main__':
    main()
