"""

Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

Ref:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame](https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""

import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import cubic_spline_planner
import racing_line
import sys
import time

SIM_LOOP = 5000

# Parameter
MAX_SPEED = 60.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 10.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 100.0  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 7.0  # maximum road width [m]
D_ROAD_W = 1.0  # road width sampling length [m]
DT = 0.2  # time tick [s]
# MAXT = 4.0  # max prediction time [m]
MINT = 2.0  # min prediction time [m]
TARGET_SPEED = 60.0 / 3.6  # target speed [m/s]
D_T_S = 10.0 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 0.2  # sampling number of target speed
ROBOT_RADIUS = 2.0  # robot radius [m]

# cost weights
KJ = 0.1
KT = 0.1
KD = 1.0
KLAT = 5.0
KLON = 3.0

show_animation = True

class quintic_polynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):

        # calc coefficient of quintic polynomial
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

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2

        return xt


class quartic_polynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, T):

        # calc coefficient of quintic polynomial
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

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class Frenet_path:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []

        self.MAXT = 4.0


def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, pred_time):

    frenet_paths = []

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in np.arange(MINT, pred_time, DT):
            fp = Frenet_path()

            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1])**2

                tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1]**2
                tfp.cv = KJ * Js + KT * Ti + KD * ds
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv

                frenet_paths.append(tfp)
        
    return frenet_paths


def calc_frenet_paths_follow_mode(c_speed, c_d, c_d_d, c_d_dd, s0, pred_time, s0_target, c_speed_target):

    frenet_paths = []
    dist_safe = 0.0
    tau = 2.0

    print(pred_time)

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in np.arange(MINT, pred_time, DT):
            fp = Frenet_path()

            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

    
            # Longitudinal motion planning (Follow Mode)
            tfp = copy.deepcopy(fp)

            #  calculate leading vehicle pos, vel, acc 
            s_lv1 = s0_target + c_speed_target * Ti
            s_lv1dot = c_speed_target
            s_lv1ddot = 0

            #  calculate target pos, vel, acc
            s_target = s_lv1 - (dist_safe + tau * s_lv1dot)
            s_targetdot = s_lv1dot
            s_targetddot = 0
            lon_qp = quintic_polynomial(s0, c_speed, 0.0, s_target, s_targetdot, 0.0, Ti)

            tfp.s = [lon_qp.calc_point(t) for t in fp.t]
            tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

            Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

            # square of diff from target speed
            ds = (TARGET_SPEED - tfp.s_d[-1])**2

            tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1]**2
            tfp.cv = KJ * Js + KT * Ti + KD * ds
            tfp.cf = KLAT * tfp.cd + KLON * tfp.cv

            frenet_paths.append(tfp)

    return frenet_paths



def calc_frenet_paths_low_velocity(c_speed, c_d, c_d_d, c_d_dd, s0, pred_time):
    frenet_paths = []

    # Longitudinal motion planning (Velocity keeping)
    for Ti in np.arange(MINT, pred_time, DT):
        tfp = Frenet_path()

        for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
            lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

            tfp.t = [t for t in np.arange(0.0, Ti, DT)]
            tfp.s = [lon_qp.calc_point(t) for t in tfp.t]
            tfp.s_d = [lon_qp.calc_first_derivative(t) for t in tfp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in tfp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in tfp.t]

            # Lateral motion planning
            for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

                fp = copy.deepcopy(tfp)
                lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

                fp.d = [lat_qp.calc_point(s) for s in tfp.s]
                fp.d_d = [lat_qp.calc_first_derivative(s) for s in tfp.s]
                fp.d_dd = [lat_qp.calc_second_derivative(s) for s in tfp.s]
                fp.d_ddd = [lat_qp.calc_third_derivative(s) for s in tfp.s]

                Jp = sum(np.power(fp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(fp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1])**2

                fp.cd = KJ * Jp + KT * Ti + KD * fp.d[-1]**2
                fp.cv = KJ * Js + KT * Ti + KD * ds
                fp.cf = KLAT * fp.cd + KLON * fp.cv

                frenet_paths.append(fp)
        
    return frenet_paths


def calc_global_paths(fplist, csp):

    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
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

    d = [((ix - ob[0])**2 + (iy - ob[1])**2)
            for (ix, iy) in zip(fp.x, fp.y)]

    collision = np.any([di <= ROBOT_RADIUS**2 for di in d])

    if collision:
        return False

    return True


def check_paths(fplist, ob):

    okind = []
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            print("velocity")
            continue
        elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # Max accel check
            print("acceleration")
            print(fplist[i].s_dd)
            continue
        elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # Max curvature check
            print("curvature")
            continue
        elif not check_collision(fplist[i], ob):
            print("collision")
            continue

        okind.append(i)
    
    return [fplist[i] for i in okind]

def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob, pred_time, s0_target, c_speed_target):

    if(c_speed == 0.0):
        print("###############")
        print("Low Velocities")
        print("###############")
        fplist = calc_frenet_paths_low_velocity(c_speed, c_d, c_d_d, c_d_dd, s0, pred_time)
        fplist = calc_global_paths(fplist, csp)
        fplist = check_paths(fplist, ob)
    else:
        print("###############")
        print("Vel keeping")
        print("###############")
        fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, pred_time)
        fplist = calc_global_paths(fplist, csp)
        fplist = check_paths(fplist, ob)
        if not fplist:
            print("###############")
            print("No trajectories, Follow Mode")
            print("###############")
            fplist = calc_frenet_paths_follow_mode(c_speed, c_d, c_d_d, c_d_dd, s0, pred_time, s0_target, c_speed_target)
            fplist = calc_global_paths(fplist, csp)
            fplist = check_paths(fplist, ob)
            
    # find minimum cost path
    mincost = float("inf")
    bestpath = None
    for fp in fplist:
        if mincost >= fp.cf:
            mincost = fp.cf
            bestpath = fp

    return bestpath

def frenet_optimal_planning_target(csp, s0, c_speed, c_d, c_d_d, c_d_dd, pred_time):

    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, pred_time)
    fplist = calc_global_paths(fplist, csp)
   
    # find minimum cost path
    mincost = float("inf")
    bestpath = None
    for fp in fplist:
        if mincost >= fp.cf:
            mincost = fp.cf
            bestpath = fp

    return bestpath


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp


def main(json_file):
    print(__file__ + " start!!")

    rc = racing_line.RacingLine

    rx,ry,insx,insy,outx,outy = rc.json_parser(json_file) 


    num_laps = 3
    wx, wy, targetx, targety = [], [], [], []

    for i in range(num_laps):
        wx.extend(rx)
        wy.extend(ry)
        targetx.extend(rx)
        targety.extend(ry)

    borders_x = insx
    borders_y = insy
    borders_x.extend(outx)
    borders_y.extend(outy)

    ob_targetx = borders_x
    ob_targety = borders_y

    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)
    tx_target, ty_target, tyaw_target, tc_target, csp_target = generate_target_course(targetx, targety)

    # initial state
    c_speed = 0.0 / 3.6  # current speed [m/s]
    c_d = 0.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.  # current course position

    c_speed_target = 10.0 / 3.6  # current speed [m/s]
    c_d_target = 0.0  # current lateral position [m]
    c_d_d_target = 0.0  # current lateral speed [m/s]
    c_d_dd_target = 0.0  # current lateral acceleration [m/s]
    s0_target = 50.0  # current course position

    area = 30.0  # animation area length [m]

    pred_time = 4.0

    for i in range(SIM_LOOP):

        # start = time.time()

        path_target = frenet_optimal_planning_target(csp_target, s0_target, 
        c_speed_target, c_d_target, c_d_d_target, c_d_dd_target, pred_time)
      
        ob_targetx.append(path_target.x[1])
        ob_targety.append(path_target.y[1])

        ob_target = np.array([ob_targetx,ob_targety])


        path = frenet_optimal_planning(
            csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob_target, pred_time, \
                s0_target, c_speed_target)

        del ob_targetx[-1]
        del ob_targety[-1]

        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]

        s0_target = path_target.s[1]
        c_d_target = path_target.d[1]
        c_d_d_target = path_target.d_d[1]
        c_d_dd_target = path_target.d_dd[1]
        c_speed_target = path_target.s_d[1]


        if(path_target.s_d[1] >= 11.0):
            c_speed_target = 10.0

        if (np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0) and (np.hypot(path_target.x[1] - tx_target[-1], path_target.y[1] - ty_target[-1]) <= 1.0):
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.plot(tx, ty)
            plt.plot(ob_target[0], ob_target[1], "xk")
            plt.plot(path.x[1:], path.y[1:], "-or")
            # plt.plot(path.x[1], path.y[1], "vc")
            circle = plt.Circle((path.x[1], path.y[1]), ROBOT_RADIUS, color='b', fill=False)
            plt.gcf().gca().add_artist(circle)
            plt.plot(path_target.x[1:], path_target.y[1:], "-ob")
            plt.plot(path_target.x[1], path_target.y[1], "vc")
            plt.xlim(path.x[1] - area, path.x[1] + area)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4] + " " + "vt[km/h]:" + str(c_speed_target * 3.6)[0:4] + " " + "a:" + str(path.s_dd[1])[0:4])
            plt.grid(True)
            plt.pause(0.0001)

        # stop = time.time()
        # duration = stop-start
        # print(duration)

    print("Finish")
    if show_animation:  # pragma: no cover
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()


if __name__ == '__main__':
    main(sys.argv[1])
