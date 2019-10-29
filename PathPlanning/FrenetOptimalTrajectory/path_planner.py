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
import racing_line
import sys

import frenet_optimal_trajectory

SIM_LOOP = 5000

# Parameter
MAX_SPEED = 60.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 10.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 100.0  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 7.0  # maximum road width [m]
D_ROAD_W = 1.0  # road width sampling length [m]
DT = 0.2  # time tick [s]
MAXT = 4.0  # max prediction time [m]
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



def calc_frenet_paths_lv(c_speed, c_d, c_d_d, c_d_dd, s0):  # Low velocities

    frenet_paths = []

    # Longitudinal motion planning 
    for Ti in np.arange(MINT, MAXT, DT):
        tfp = frenet_optimal_trajectory.Frenet_path()

        for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
            lon_qp = frenet_optimal_trajectory.quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

            tfp.t = [t for t in np.arange(0.0, Ti, DT)]
            tfp.s = [lon_qp.calc_point(t) for t in tfp.t]
            tfp.s_d = [lon_qp.calc_first_derivative(t) for t in tfp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in tfp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in tfp.t]

            # Lateral motion planning
            for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

                fp = copy.deepcopy(tfp)
                lat_qp = frenet_optimal_trajectory.quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

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

def calc_frenet_paths_fm(c_speed, c_d, c_d_d, c_d_dd, s0, s0_target, c_speed_target):  # Follow mode

    frenet_paths = []
    dist_safe = 0.0
    tau = 2.0
    
    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in np.arange(MINT, MAXT, DT):
            fp = frenet_optimal_trajectory.Frenet_path()

            lat_qp = frenet_optimal_trajectory.quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

    
            # Longitudinal motion planning
            tfp = copy.deepcopy(fp)

            #  calculate leading vehicle pos, vel, acc 
            s_lv1 = s0_target + c_speed_target * Ti
            s_lv1dot = c_speed_target
            s_lv1ddot = 0.0

            #  calculate target pos, vel, acc
            s_target = s_lv1 - (dist_safe + tau * s_lv1dot)
            s_targetdot = s_lv1dot
            s_targetddot = 0.0
            lon_qp = frenet_optimal_trajectory.quintic_polynomial(s0, c_speed, s_lv1ddot, s_target, s_targetdot, s_targetddot, Ti)

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



def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob, s0_target, c_speed_target):


    if(c_speed == 0.0):
        print("###############")
        print("Low Velocities")
        fplist = calc_frenet_paths_lv(c_speed, c_d, c_d_d, c_d_dd, s0)
        fplist = frenet_optimal_trajectory.calc_global_paths(fplist, csp)
        fplist = frenet_optimal_trajectory.check_paths(fplist, ob)
    else:
        print("###############")
        print("Velocity keeping")
        fplist = frenet_optimal_trajectory.calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
        fplist = frenet_optimal_trajectory.calc_global_paths(fplist, csp)
        fplist = frenet_optimal_trajectory.check_paths(fplist, ob)
        if not fplist:
            print("###############")
            print("No trajectories, Follow Mode")
            fplist = calc_frenet_paths_fm(c_speed, c_d, c_d_d, c_d_dd, s0, s0_target, c_speed_target)
            fplist = frenet_optimal_trajectory.calc_global_paths(fplist, csp)
            fplist = frenet_optimal_trajectory.check_paths(fplist, ob)
            
    # find minimum cost path
    mincost = float("inf")
    bestpath = None
    for fp in fplist:
        if mincost >= fp.cf:
            mincost = fp.cf
            bestpath = fp

    return bestpath




def main(json_file):
    print(__file__ + " start!!")

    rc = racing_line.RacingLine()

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

    ob_borders = np.array([borders_x, borders_y])

    tx, ty, tyaw, tc, csp = frenet_optimal_trajectory.generate_target_course(wx, wy)
    tx_target, ty_target, tyaw_target, tc_target, csp_target = frenet_optimal_trajectory.generate_target_course(targetx, targety)

    # initial state
    c_speed = 0.0 / 3.6  # current speed [m/s]
    c_d = 0.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    c_speed_target = 0.0 / 3.6  # current speed [m/s]
    c_d_target = 0.0  # current lateral position [m]
    c_d_d_target = 0.0  # current lateral speed [m/s]
    c_d_dd_target = 0.0  # current lateral acceleration [m/s]
    s0_target = 50.0  # current course position

    area = 30.0  # animation area length [m]

    for i in range(SIM_LOOP):

        path_target = frenet_optimal_planning(csp_target, s0_target, c_speed_target, c_d_target, c_d_d_target, c_d_dd_target, ob_borders, 0.0, 0.0)
      
        ob_targetx.append(path_target.x[1])
        ob_targety.append(path_target.y[1])

        ob_target = np.array([ob_targetx,ob_targety])

        path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob_target, s0_target, c_speed_target)

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


        if(path_target.s_d[1] >= 11.0): # just for simulation purpose
            c_speed_target = 10.0

        if (np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0) and (np.hypot(path_target.x[1] - tx_target[-1], path_target.y[1] - ty_target[-1]) <= 1.0):
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.plot(tx, ty)
            plt.plot(ob_target[0], ob_target[1], "xk")
            plt.plot(path.x[1:], path.y[1:], "-or")
            circle = plt.Circle((path.x[1], path.y[1]), ROBOT_RADIUS, color='b', fill=False)
            plt.gcf().gca().add_artist(circle)
            plt.plot(path_target.x[1:], path_target.y[1:], "-ob")
            plt.plot(path_target.x[1], path_target.y[1], "vc")
            plt.xlim(path.x[1] - area, path.x[1] + area)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4] + " " + "vt[km/h]:" + str(c_speed_target * 3.6)[0:4] + " " + "a:" + str(path.s_dd[1])[0:4])
            plt.grid(True)
            plt.pause(0.0001)


    print("Finish")
    if show_animation:  # pragma: no cover
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()


if __name__ == '__main__':
    main(sys.argv[1])
