"""

Dubins path planner sample code

author Atsushi Sakai(@Atsushi_twi)

"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../utils/")

import math
import numpy as np
from utils.angle import angle_mod, create_2d_rotation_matrix

show_animation = True


def plan_dubins_path(s_x, s_y, s_yaw,
                     g_x, g_y, g_yaw,
                     curvature,
                     step_size=0.1):
    """
    Path dubins path

    Parameters
    ----------
    s_x : float
        x position of the start point [m]
    s_y : float
        y position of the start point [m]
    s_yaw : float
        yaw angle of the start point [rad]
    g_x : float
        x position of the goal point [m]
    g_y : float
        y position of the end point [m]
    g_yaw : float
        yaw angle of the end point [rad]
    curvature : float
        curvature for curve [1/m]
    step_size : float (optional)
        step size between two path points [m]. Default is 0.1

    Returns
    -------
    x_list: array
        x positions of the path
    y_list: array
        y positions of the path
    yaw_list: array
        yaw angles of the path
    modes: array
        mode list of the path
    lengths: array
        length list of the path segments.

    """
    # calculate local goal x, y, yaw
    l_rot = create_2d_rotation_matrix(s_yaw)
    le_xy = np.stack([g_x - s_x, g_y - s_y]).T @ l_rot
    local_goal_x = le_xy[0]
    local_goal_y = le_xy[1]
    local_goal_yaw = g_yaw - s_yaw

    lp_x, lp_y, lp_yaw, modes, lengths = dubins_path_planning_from_origin(
        local_goal_x, local_goal_y, local_goal_yaw, curvature, step_size)

    # Convert a local coordinate path to the global coordinate
    rot = create_2d_rotation_matrix(-s_yaw)
    converted_xy = np.stack([lp_x, lp_y]).T @ rot
    x_list = converted_xy[:, 0] + s_x
    y_list = converted_xy[:, 1] + s_y
    yaw_list = angle_mod(np.array(lp_yaw) + s_yaw)

    return x_list, y_list, yaw_list, modes, lengths


def _mod2pi(theta):
    return angle_mod(theta, zero_2_2pi=True)


def _LSL(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp0 = d + sa - sb

    mode = ["L", "S", "L"]
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb))
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = math.atan2((cb - ca), tmp0)
    t = _mod2pi(-alpha + tmp1)
    p = math.sqrt(p_squared)
    q = _mod2pi(beta - tmp1)

    return t, p, q, mode


def right_straight_right(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp0 = d - sa + sb
    mode = ["R", "S", "R"]
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa))
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = math.atan2((ca - cb), tmp0)
    t = angle_mod(alpha - tmp1, zero_2_2pi=True)
    p = math.sqrt(p_squared)
    q = _mod2pi(-beta + tmp1)

    return t, p, q, mode


def left_straight_right(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb))
    mode = ["L", "S", "R"]
    if p_squared < 0:
        return None, None, None, mode
    p = math.sqrt(p_squared)
    tmp2 = math.atan2((-ca - cb), (d + sa + sb)) - math.atan2(-2.0, p)
    t = _mod2pi(-alpha + tmp2)
    q = _mod2pi(-_mod2pi(beta) + tmp2)

    return t, p, q, mode


def right_straight_left(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb))
    mode = ["R", "S", "L"]
    if p_squared < 0:
        return None, None, None, mode
    p = math.sqrt(p_squared)
    tmp2 = math.atan2((ca + cb), (d - sa - sb)) - math.atan2(2.0, p)
    t = _mod2pi(alpha - tmp2)
    q = _mod2pi(beta - tmp2)

    return t, p, q, mode


def right_left_right(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    mode = ["R", "L", "R"]
    tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
    if abs(tmp_rlr) > 1.0:
        return None, None, None, mode

    p = _mod2pi(2 * math.pi - math.acos(tmp_rlr))
    t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + _mod2pi(p / 2.0))
    q = _mod2pi(alpha - beta - t + _mod2pi(p))
    return t, p, q, mode


def _LRL(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    mode = ["L", "R", "L"]
    tmp_lrl = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (- sa + sb)) / 8.0
    if abs(tmp_lrl) > 1:
        return None, None, None, mode
    p = _mod2pi(2 * math.pi - math.acos(tmp_lrl))
    t = _mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
    q = _mod2pi(_mod2pi(beta) - alpha - t + _mod2pi(p))

    return t, p, q, mode


def dubins_path_planning_from_origin(end_x, end_y, end_yaw, curvature,
                                     step_size):
    dx = end_x
    dy = end_y
    D = math.hypot(dx, dy)
    d = D * curvature

    theta = _mod2pi(math.atan2(dy, dx))
    alpha = _mod2pi(- theta)
    beta = _mod2pi(end_yaw - theta)

    planning_funcs = [_LSL, right_straight_right,
                      left_straight_right, right_straight_left,
                      right_left_right, _LRL]

    best_cost = float("inf")
    bt, bp, bq, best_mode = None, None, None, None

    for planner in planning_funcs:
        t, p, q, mode = planner(alpha, beta, d)
        if t is None:
            continue

        cost = (abs(t) + abs(p) + abs(q))
        if best_cost > cost:
            bt, bp, bq, best_mode = t, p, q, mode
            best_cost = cost
    lengths = [bt, bp, bq]

    x_list, y_list, yaw_list, directions = generate_local_course(sum(lengths),
                                                                 lengths,
                                                                 best_mode,
                                                                 curvature,
                                                                 step_size)

    lengths = [length / curvature for length in lengths]

    return x_list, y_list, yaw_list, best_mode, lengths


def interpolate(ind, length, mode, max_curvature, origin_x, origin_y,
                origin_yaw, path_x, path_y, path_yaw, directions):
    if mode == "S":
        path_x[ind] = origin_x + length / max_curvature * math.cos(origin_yaw)
        path_y[ind] = origin_y + length / max_curvature * math.sin(origin_yaw)
        path_yaw[ind] = origin_yaw
    else:  # curve
        ldx = math.sin(length) / max_curvature
        ldy = 0.0
        if mode == "L":  # left turn
            ldy = (1.0 - math.cos(length)) / max_curvature
        elif mode == "R":  # right turn
            ldy = (1.0 - math.cos(length)) / -max_curvature
        gdx = math.cos(-origin_yaw) * ldx + math.sin(-origin_yaw) * ldy
        gdy = -math.sin(-origin_yaw) * ldx + math.cos(-origin_yaw) * ldy
        path_x[ind] = origin_x + gdx
        path_y[ind] = origin_y + gdy

    if mode == "L":  # left turn
        path_yaw[ind] = origin_yaw + length
    elif mode == "R":  # right turn
        path_yaw[ind] = origin_yaw - length

    if length > 0.0:
        directions[ind] = 1
    else:
        directions[ind] = -1

    return path_x, path_y, path_yaw, directions


def generate_local_course(total_length, lengths, modes, max_curvature,
                          step_size):
    n_point = math.trunc(total_length / step_size) + len(lengths) + 4

    p_x = [0.0 for _ in range(n_point)]
    p_y = [0.0 for _ in range(n_point)]
    p_yaw = [0.0 for _ in range(n_point)]
    directions = [0.0 for _ in range(n_point)]
    ind = 1

    if lengths[0] > 0.0:
        directions[0] = 1
    else:
        directions[0] = -1

    ll = 0.0

    for (m, length, i) in zip(modes, lengths, range(len(modes))):
        if length == 0.0:
            continue
        elif length > 0.0:
            dist = step_size
        else:
            dist = -step_size

        # set origin state
        origin_x, origin_y, origin_yaw = p_x[ind], p_y[ind], p_yaw[ind]

        ind -= 1
        if i >= 1 and (lengths[i - 1] * lengths[i]) > 0:
            pd = - dist - ll
        else:
            pd = dist - ll

        while abs(pd) <= abs(length):
            ind += 1
            p_x, p_y, p_yaw, directions = interpolate(ind, pd, m,
                                                      max_curvature,
                                                      origin_x,
                                                      origin_y,
                                                      origin_yaw,
                                                      p_x, p_y,
                                                      p_yaw,
                                                      directions)
            pd += dist

        ll = length - pd - dist  # calc remain length

        ind += 1
        p_x, p_y, p_yaw, directions = interpolate(ind, length, m,
                                                  max_curvature,
                                                  origin_x, origin_y,
                                                  origin_yaw,
                                                  p_x, p_y, p_yaw,
                                                  directions)

    if len(p_x) <= 1:
        return [], [], [], []

    # remove unused data
    while len(p_x) >= 1 and p_x[-1] == 0.0:
        p_x.pop()
        p_y.pop()
        p_yaw.pop()
        directions.pop()

    return p_x, p_y, p_yaw, directions


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r",
               ec="k"):  # pragma: no cover
    import matplotlib.pyplot as plt
    if not isinstance(x, float):
        for (i_x, i_y, i_yaw) in zip(x, y, yaw):
            plot_arrow(i_x, i_y, i_yaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw), fc=fc,
                  ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def main():
    print("Dubins path planner sample start!!")
    import matplotlib.pyplot as plt

    start_x = 1.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = np.deg2rad(45.0)  # [rad]

    end_x = -3.0  # [m]
    end_y = -3.0  # [m]
    end_yaw = np.deg2rad(-45.0)  # [rad]

    curvature = 1.0

    path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(start_x,
                                                               start_y,
                                                               start_yaw,
                                                               end_x,
                                                               end_y,
                                                               end_yaw,
                                                               curvature)

    if show_animation:
        plt.plot(path_x, path_y, label="final course " + "".join(mode))

        # plotting
        plot_arrow(start_x, start_y, start_yaw)
        plot_arrow(end_x, end_y, end_yaw)

        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()


if __name__ == '__main__':
    main()
