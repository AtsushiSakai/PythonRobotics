"""

Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

Reference:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
(https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""

import numpy as np
import matplotlib.pyplot as plt
import copy
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent))

from QuinticPolynomialsPlanner.quintic_polynomials_planner import QuinticPolynomial
from CubicSpline import cubic_spline_planner

from enum import Enum, auto
from FrenetOptimalTrajectory.cartesian_frenet_converter import (
    CartesianFrenetConverter,
)


class LateralMovement(Enum):
    HIGH_SPEED = auto()
    LOW_SPEED = auto()


class LongitudinalMovement(Enum):
    MERGING_AND_STOPPING = auto()
    VELOCITY_KEEPING = auto()


# Default Parameters

LATERAL_MOVEMENT = LateralMovement.HIGH_SPEED
LONGITUDINAL_MOVEMENT = LongitudinalMovement.VELOCITY_KEEPING

MAX_SPEED = 50.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 5.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 1.0  # maximum curvature [1/m]
DT = 0.2  # time tick [s]
MAX_T = 5.0  # max prediction time [m]
MIN_T = 4.0  # min prediction time [m]
N_S_SAMPLE = 1  # sampling number of target speed

# cost weights
K_J = 0.1
K_T = 0.1
K_S_DOT = 1.0
K_D = 1.0
K_S = 1.0
K_LAT = 1.0
K_LON = 1.0

SIM_LOOP = 500
show_animation = True


if LATERAL_MOVEMENT == LateralMovement.LOW_SPEED:
    MAX_ROAD_WIDTH = 1.0  # maximum road width [m]
    D_ROAD_W = 0.2  # road width sampling length [m]
    TARGET_SPEED = 3.0 / 3.6  # maximum speed [m/s]
    D_T_S = 0.5 / 3.6  # target speed sampling length [m/s]
    # Waypoints
    WX = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0]
    WY = [0.0, 0.0, 1.0, 0.0, -1.0, -2.0]
    OBSTACLES = np.array([[3.0, 1.0], [5.0, -0.0], [6.0, 0.5], [8.0, -1.5]])
    ROBOT_RADIUS = 0.5  # robot radius [m]

    # Initial state parameters
    INITIAL_SPEED = 1.0 / 3.6  # current speed [m/s]
    INITIAL_ACCEL = 0.0  # current acceleration [m/ss]
    INITIAL_LAT_POSITION = 0.5  # current lateral position [m]
    INITIAL_LAT_SPEED = 0.0  # current lateral speed [m/s]
    INITIAL_LAT_ACCELERATION = 0.0  # current lateral acceleration [m/s]
    INITIAL_COURSE_POSITION = 0.0  # current course position

    ANIMATION_AREA = 5.0  # Animation area length [m]

    STOP_S = 4.0  # Merge and stop distance [m]
    D_S = 0.3  # Stop point sampling length [m]
    N_STOP_S_SAMPLE = 3  # Stop point sampling number
else:
    MAX_ROAD_WIDTH = 7.0  # maximum road width [m]
    D_ROAD_W = 1.0  # road width sampling length [m]
    TARGET_SPEED = 30.0 / 3.6  # target speed [m/s]
    D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
    # Waypoints
    WX = [0.0, 10.0, 20.5, 35.0, 70.5]
    WY = [0.0, -6.0, 5.0, 6.5, 0.0]
    # Obstacle list
    OBSTACLES = np.array(
        [[20.0, 10.0], [30.0, 6.0], [30.0, 8.0], [35.0, 8.0], [50.0, 3.0]]
    )
    ROBOT_RADIUS = 2.0  # robot radius [m]

    # Initial state parameters
    INITIAL_SPEED = 10.0 / 3.6  # current speed [m/s]
    INITIAL_ACCEL = 0.0  # current acceleration [m/ss]
    INITIAL_LAT_POSITION = 2.0  # current lateral position [m]
    INITIAL_LAT_SPEED = 0.0  # current lateral speed [m/s]
    INITIAL_LAT_ACCELERATION = 0.0  # current lateral acceleration [m/s]
    INITIAL_COURSE_POSITION = 0.0  # current course position

    ANIMATION_AREA = 20.0  # Animation area length [m]
    STOP_S = 25.0  # Merge and stop distance [m]
    D_S = 2  # Stop point sampling length [m]
    N_STOP_S_SAMPLE = 4  # Stop point sampling number


class LateralMovementStrategy:
    def calc_lateral_trajectory(self, fp, di, c_d, c_d_d, c_d_dd, Ti):
        """
        Calculate the lateral trajectory
        """
        raise NotImplementedError("calc_lateral_trajectory not implemented")

    def calc_cartesian_parameters(self, fp, csp):
        """
        Calculate the cartesian parameters (x, y, yaw, curvature, v, a)
        """
        raise NotImplementedError("calc_cartesian_parameters not implemented")


class HighSpeedLateralMovementStrategy(LateralMovementStrategy):
    def calc_lateral_trajectory(self, fp, di, c_d, c_d_d, c_d_dd, Ti):
        tp = copy.deepcopy(fp)
        s0_d = fp.s_d[0]
        s0_dd = fp.s_dd[0]
        # d'(t) = d'(s) * s'(t)
        # d''(t) = d''(s) * s'(t)^2 + d'(s) * s''(t)
        lat_qp = QuinticPolynomial(
            c_d, c_d_d * s0_d, c_d_dd * s0_d**2 + c_d_d * s0_dd, di, 0.0, 0.0, Ti
        )

        tp.d = []
        tp.d_d = []
        tp.d_dd = []
        tp.d_ddd = []

        # Calculate all derivatives in a single loop to reduce iterations
        for i in range(len(fp.t)):
            t = fp.t[i]
            s_d = fp.s_d[i]
            s_dd = fp.s_dd[i]

            s_d_inv = 1.0 / (s_d + 1e-6) + 1e-6  # Avoid division by zero
            s_d_inv_sq = s_d_inv * s_d_inv  # Square of inverse

            d = lat_qp.calc_point(t)
            d_d = lat_qp.calc_first_derivative(t)
            d_dd = lat_qp.calc_second_derivative(t)
            d_ddd = lat_qp.calc_third_derivative(t)

            tp.d.append(d)
            # d'(s) = d'(t) / s'(t)
            tp.d_d.append(d_d * s_d_inv)
            # d''(s) = (d''(t) - d'(s) * s''(t)) / s'(t)^2
            tp.d_dd.append((d_dd - tp.d_d[i] * s_dd) * s_d_inv_sq)
            tp.d_ddd.append(d_ddd)

        return tp

    def calc_cartesian_parameters(self, fp, csp):
        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = csp.calc_yaw(fp.s[i])
            i_kappa = csp.calc_curvature(fp.s[i])
            i_dkappa = csp.calc_curvature_rate(fp.s[i])
            s_condition = [fp.s[i], fp.s_d[i], fp.s_dd[i]]
            d_condition = [
                fp.d[i],
                fp.d_d[i],
                fp.d_dd[i],
            ]
            x, y, theta, kappa, v, a = CartesianFrenetConverter.frenet_to_cartesian(
                fp.s[i], ix, iy, i_yaw, i_kappa, i_dkappa, s_condition, d_condition
            )
            fp.x.append(x)
            fp.y.append(y)
            fp.yaw.append(theta)
            fp.c.append(kappa)
            fp.v.append(v)
            fp.a.append(a)
        return fp


class LowSpeedLateralMovementStrategy(LateralMovementStrategy):
    def calc_lateral_trajectory(self, fp, di, c_d, c_d_d, c_d_dd, Ti):
        s0 = fp.s[0]
        s1 = fp.s[-1]
        tp = copy.deepcopy(fp)
        # d = d(s), d_d = d'(s), d_dd = d''(s)
        # * shift s range from [s0, s1] to [0, s1 - s0]
        lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, s1 - s0)

        tp.d = [lat_qp.calc_point(s - s0) for s in fp.s]
        tp.d_d = [lat_qp.calc_first_derivative(s - s0) for s in fp.s]
        tp.d_dd = [lat_qp.calc_second_derivative(s - s0) for s in fp.s]
        tp.d_ddd = [lat_qp.calc_third_derivative(s - s0) for s in fp.s]
        return tp

    def calc_cartesian_parameters(self, fp, csp):
        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = csp.calc_yaw(fp.s[i])
            i_kappa = csp.calc_curvature(fp.s[i])
            i_dkappa = csp.calc_curvature_rate(fp.s[i])
            s_condition = [fp.s[i], fp.s_d[i], fp.s_dd[i]]
            d_condition = [fp.d[i], fp.d_d[i], fp.d_dd[i]]
            x, y, theta, kappa, v, a = CartesianFrenetConverter.frenet_to_cartesian(
                fp.s[i], ix, iy, i_yaw, i_kappa, i_dkappa, s_condition, d_condition
            )
            fp.x.append(x)
            fp.y.append(y)
            fp.yaw.append(theta)
            fp.c.append(kappa)
            fp.v.append(v)
            fp.a.append(a)
        return fp


class LongitudinalMovementStrategy:
    def calc_longitudinal_trajectory(self, c_speed, c_accel, Ti, s0):
        """
        Calculate the longitudinal trajectory
        """
        raise NotImplementedError("calc_longitudinal_trajectory not implemented")

    def get_d_arrange(self, s0):
        """
        Get the d sample range
        """
        raise NotImplementedError("get_d_arrange not implemented")

    def calc_destination_cost(self, fp):
        """
        Calculate the destination cost
        """
        raise NotImplementedError("calc_destination_cost not implemented")


class VelocityKeepingLongitudinalMovementStrategy(LongitudinalMovementStrategy):
    def calc_longitudinal_trajectory(self, c_speed, c_accel, Ti, s0):
        fplist = []
        for tv in np.arange(
            TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S
        ):
            fp = FrenetPath()
            lon_qp = QuarticPolynomial(s0, c_speed, c_accel, tv, 0.0, Ti)
            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.s = [lon_qp.calc_point(t) for t in fp.t]
            fp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
            fp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
            fp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]
            fplist.append(fp)
        return fplist

    def get_d_arrange(self, s0):
        return np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W)

    def calc_destination_cost(self, fp):
        ds = (TARGET_SPEED - fp.s_d[-1]) ** 2
        return K_S_DOT * ds


class MergingAndStoppingLongitudinalMovementStrategy(LongitudinalMovementStrategy):
    def calc_longitudinal_trajectory(self, c_speed, c_accel, Ti, s0):
        if s0 >= STOP_S:
            return []
        fplist = []
        for s in np.arange(
            STOP_S - D_S * N_STOP_S_SAMPLE, STOP_S + D_S * N_STOP_S_SAMPLE, D_S
        ):
            fp = FrenetPath()
            lon_qp = QuinticPolynomial(s0, c_speed, c_accel, s, 0.0, 0.0, Ti)
            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.s = [lon_qp.calc_point(t) for t in fp.t]
            fp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
            fp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
            fp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]
            fplist.append(fp)
        return fplist

    def get_d_arrange(self, s0):
        # Only if s0 is less than STOP_S / 3, then we sample the road width
        if s0 < STOP_S / 3:
            return np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W)
        else:
            return [0.0]

    def calc_destination_cost(self, fp):
        ds = (STOP_S - fp.s[-1]) ** 2
        return K_S * ds

LATERAL_MOVEMENT_STRATEGY: LateralMovementStrategy
LONGITUDINAL_MOVEMENT_STRATEGY: LongitudinalMovementStrategy

if LATERAL_MOVEMENT == LateralMovement.HIGH_SPEED:
    LATERAL_MOVEMENT_STRATEGY = HighSpeedLateralMovementStrategy()
else:
    LATERAL_MOVEMENT_STRATEGY = LowSpeedLateralMovementStrategy()

if LONGITUDINAL_MOVEMENT == LongitudinalMovement.VELOCITY_KEEPING:
    LONGITUDINAL_MOVEMENT_STRATEGY = VelocityKeepingLongitudinalMovementStrategy()
else:
    LONGITUDINAL_MOVEMENT_STRATEGY = MergingAndStoppingLongitudinalMovementStrategy()


class QuarticPolynomial:
    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time**2, 4 * time**3], [6 * time, 12 * time**2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time, axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2 + 4 * self.a4 * t**3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class FrenetPath:
    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []  # d'(s)
        self.d_dd = []  # d''(s)
        self.d_ddd = []  # d'''(t) in low speed / d'''(s) in high speed
        self.s = []
        self.s_d = []  # s'(t)
        self.s_dd = []  # s''(t)
        self.s_ddd = []  # s'''(t)
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.a = []
        self.ds = []
        self.c = []

    def pop_front(self):
        self.x.pop(0)
        self.y.pop(0)
        self.yaw.pop(0)
        self.v.pop(0)
        self.a.pop(0)
        self.s.pop(0)
        self.s_d.pop(0)
        self.s_dd.pop(0)
        self.s_ddd.pop(0)
        self.d.pop(0)
        self.d_d.pop(0)
        self.d_dd.pop(0)
        self.d_ddd.pop(0)


def calc_frenet_paths(c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []

    for Ti in np.arange(MIN_T, MAX_T, DT):
        lon_paths = LONGITUDINAL_MOVEMENT_STRATEGY.calc_longitudinal_trajectory(
            c_s_d, c_s_dd, Ti, s0
        )

        for fp in lon_paths:
            for di in LONGITUDINAL_MOVEMENT_STRATEGY.get_d_arrange(s0):
                tp = LATERAL_MOVEMENT_STRATEGY.calc_lateral_trajectory(
                    fp, di, c_d, c_d_d, c_d_dd, Ti
                )

                Jp = sum(np.power(tp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tp.s_ddd, 2))  # square of jerk

                lat_cost = K_J * Jp + K_T * Ti + K_D * tp.d[-1] ** 2
                lon_cost = (
                    K_J * Js
                    + K_T * Ti
                    + LONGITUDINAL_MOVEMENT_STRATEGY.calc_destination_cost(tp)
                )
                tp.cf = K_LAT * lat_cost + K_LON * lon_cost
                frenet_paths.append(tp)

    return frenet_paths


def calc_global_paths(fplist, csp):
    return [
        LATERAL_MOVEMENT_STRATEGY.calc_cartesian_parameters(fp, csp) for fp in fplist
    ]


def check_collision(fp, ob):
    for i in range(len(ob[:, 0])):
        d = [
            ((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
            for (ix, iy) in zip(fp.x, fp.y)
        ]

        collision = any([di <= ROBOT_RADIUS**2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):
    path_dict = {
        "max_speed_error": [],
        "max_accel_error": [],
        "max_curvature_error": [],
        "collision_error": [],
        "ok": [],
    }
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].v]):  # Max speed check
            path_dict["max_speed_error"].append(fplist[i])
        elif any([abs(a) > MAX_ACCEL for a in fplist[i].a]):  # Max accel check
            path_dict["max_accel_error"].append(fplist[i])
        elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # Max curvature check
            path_dict["max_curvature_error"].append(fplist[i])
        elif not check_collision(fplist[i], ob):
            path_dict["collision_error"].append(fplist[i])
        else:
            path_dict["ok"].append(fplist[i])
    return path_dict


def frenet_optimal_planning(csp, s0, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, ob):
    fplist = calc_frenet_paths(c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    fpdict = check_paths(fplist, ob)

    # find minimum cost path
    min_cost = float("inf")
    best_path = None
    for fp in fpdict["ok"]:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp

    return [best_path, fpdict]


def generate_target_course(x, y):
    csp = cubic_spline_planner.CubicSpline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp


def main():
    print(__file__ + " start!!")

    tx, ty, tyaw, tc, csp = generate_target_course(WX, WY)

    # Initialize state using global parameters
    c_s_d = INITIAL_SPEED
    c_s_dd = INITIAL_ACCEL
    c_d = INITIAL_LAT_POSITION
    c_d_d = INITIAL_LAT_SPEED
    c_d_dd = INITIAL_LAT_ACCELERATION
    s0 = INITIAL_COURSE_POSITION

    area = ANIMATION_AREA

    last_path = None

    for i in range(SIM_LOOP):
        [path, fpdict] = frenet_optimal_planning(
            csp, s0, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, OBSTACLES
        )

        if path is None:
            path = copy.deepcopy(last_path)
            path.pop_front()
        if len(path.x) <= 1:
            print("Finish")
            break

        last_path = path
        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_s_d = path.s_d[1]
        c_s_dd = path.s_dd[1]
        if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )
            plt.plot(tx, ty)
            plt.plot(OBSTACLES[:, 0], OBSTACLES[:, 1], "xk")
            plt.plot(path.x[1:], path.y[1:], "-or")
            plt.plot(path.x[1], path.y[1], "vc")
            plt.xlim(path.x[1] - area, path.x[1] + area)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[km/h]:" + str(path.v[1] * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.0001)

    print("Finish")
    if show_animation:  # pragma: no cover
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()


if __name__ == "__main__":
    main()
