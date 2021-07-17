"""

Reeds Shepp path planner sample code

author Atsushi Sakai(@Atsushi_twi)

"""
import math

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class Path:
    """
    Path data container
    """

    def __init__(self):
        # course segment length  (negative value is backward segment)
        self.lengths = []
        # course segment type char ("S": straight, "L": left, "R": right)
        self.ctypes = []
        self.L = 0.0  # Total lengths of the path
        self.x = []  # x positions
        self.y = []  # y positions
        self.yaw = []  # orientations [rad]
        self.directions = []  # directions (1:forward, -1:backward)


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    if isinstance(x, list):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw), fc=fc,
                  ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def mod2pi(x):
    # Be consistent with fmod in cplusplus here.
    v = np.mod(x, np.copysign(2.0 * math.pi, x))
    if v < -math.pi:
        v += 2.0 * math.pi
    else:
        if v > math.pi:
            v -= 2.0 * math.pi
    return v


def straight_left_straight(x, y, phi):
    phi = mod2pi(phi)
    if y > 0.0 and 0.0 < phi < math.pi * 0.99:
        xd = - y / math.tan(phi) + x
        t = xd - math.tan(phi / 2.0)
        u = phi
        v = math.sqrt((x - xd) ** 2 + y ** 2) - math.tan(phi / 2.0)
        return True, t, u, v
    elif y < 0.0 < phi < math.pi * 0.99:
        xd = - y / math.tan(phi) + x
        t = xd - math.tan(phi / 2.0)
        u = phi
        v = -math.sqrt((x - xd) ** 2 + y ** 2) - math.tan(phi / 2.0)
        return True, t, u, v

    return False, 0.0, 0.0, 0.0


def set_path(paths, lengths, ctypes, step_size):
    path = Path()
    path.ctypes = ctypes
    path.lengths = lengths
    path.L = sum(np.abs(lengths))

    # check same path exist
    for i_path in paths:
        type_is_same = (i_path.ctypes == path.ctypes)
        length_is_close = (sum(np.abs(i_path.lengths)) - path.L) <= step_size
        if type_is_same and length_is_close:
            return paths  # same path found, so do not insert path

    # check path is long enough
    if path.L <= step_size:
        return paths  # too short, so do not insert path

    paths.append(path)
    return paths


def straight_curve_straight(x, y, phi, paths, step_size):
    flag, t, u, v = straight_left_straight(x, y, phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["S", "L", "S"], step_size)

    flag, t, u, v = straight_left_straight(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["S", "R", "S"], step_size)

    return paths


def polar(x, y):
    r = math.sqrt(x ** 2 + y ** 2)
    theta = math.atan2(y, x)
    return r, theta


def left_straight_left(x, y, phi):
    u, t = polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if t >= 0.0:
        v = mod2pi(phi - t)
        if v >= 0.0:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0


def left_right_left(x, y, phi):
    u1, t1 = polar(x - math.sin(phi), y - 1.0 + math.cos(phi))

    if u1 <= 4.0:
        u = -2.0 * math.asin(0.25 * u1)
        t = mod2pi(t1 + 0.5 * u + math.pi)
        v = mod2pi(phi - t + u)

        if t >= 0.0 >= u:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0


def curve_curve_curve(x, y, phi, paths, step_size):
    flag, t, u, v = left_right_left(x, y, phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["L", "R", "L"], step_size)

    flag, t, u, v = left_right_left(-x, y, -phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["L", "R", "L"], step_size)

    flag, t, u, v = left_right_left(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["R", "L", "R"], step_size)

    flag, t, u, v = left_right_left(-x, -y, phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R", "L", "R"], step_size)

    # backwards
    xb = x * math.cos(phi) + y * math.sin(phi)
    yb = x * math.sin(phi) - y * math.cos(phi)

    flag, t, u, v = left_right_left(xb, yb, phi)
    if flag:
        paths = set_path(paths, [v, u, t], ["L", "R", "L"], step_size)

    flag, t, u, v = left_right_left(-xb, yb, -phi)
    if flag:
        paths = set_path(paths, [-v, -u, -t], ["L", "R", "L"], step_size)

    flag, t, u, v = left_right_left(xb, -yb, -phi)
    if flag:
        paths = set_path(paths, [v, u, t], ["R", "L", "R"], step_size)

    flag, t, u, v = left_right_left(-xb, -yb, phi)
    if flag:
        paths = set_path(paths, [-v, -u, -t], ["R", "L", "R"], step_size)

    return paths


def curve_straight_curve(x, y, phi, paths, step_size):
    flag, t, u, v = left_straight_left(x, y, phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["L", "S", "L"], step_size)

    flag, t, u, v = left_straight_left(-x, y, -phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["L", "S", "L"], step_size)

    flag, t, u, v = left_straight_left(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["R", "S", "R"], step_size)

    flag, t, u, v = left_straight_left(-x, -y, phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R", "S", "R"], step_size)

    flag, t, u, v = left_straight_right(x, y, phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["L", "S", "R"], step_size)

    flag, t, u, v = left_straight_right(-x, y, -phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["L", "S", "R"], step_size)

    flag, t, u, v = left_straight_right(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["R", "S", "L"], step_size)

    flag, t, u, v = left_straight_right(-x, -y, phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R", "S", "L"], step_size)

    return paths


def left_straight_right(x, y, phi):
    u1, t1 = polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    u1 = u1 ** 2
    if u1 >= 4.0:
        u = math.sqrt(u1 - 4.0)
        theta = math.atan2(2.0, u)
        t = mod2pi(t1 + theta)
        v = mod2pi(t - phi)

        if t >= 0.0 and v >= 0.0:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0


def generate_path(q0, q1, max_curvature, step_size):
    dx = q1[0] - q0[0]
    dy = q1[1] - q0[1]
    dth = q1[2] - q0[2]
    c = math.cos(q0[2])
    s = math.sin(q0[2])
    x = (c * dx + s * dy) * max_curvature
    y = (-s * dx + c * dy) * max_curvature

    paths = []
    paths = straight_curve_straight(x, y, dth, paths, step_size)
    paths = curve_straight_curve(x, y, dth, paths, step_size)
    paths = curve_curve_curve(x, y, dth, paths, step_size)

    return paths


def calc_interpolate_dists_list(lengths, step_size):
    interpolate_dists_list = []
    for length in lengths:
        d_dist = step_size if length >= 0.0 else -step_size
        interp_dists = np.arange(0.0, length, d_dist)
        interp_dists = np.append(interp_dists, length)
        interpolate_dists_list.append(interp_dists)

    return interpolate_dists_list


def generate_local_course(lengths, modes, max_curvature, step_size):
    interpolate_dists_list = calc_interpolate_dists_list(lengths, step_size)

    origin_x, origin_y, origin_yaw = 0.0, 0.0, 0.0

    xs, ys, yaws, directions = [], [], [], []
    for (interp_dists, mode, length) in zip(interpolate_dists_list, modes,
                                            lengths):

        for dist in interp_dists:
            x, y, yaw, direction = interpolate(dist, length, mode,
                                               max_curvature, origin_x,
                                               origin_y, origin_yaw)
            xs.append(x)
            ys.append(y)
            yaws.append(yaw)
            directions.append(direction)
        origin_x = xs[-1]
        origin_y = ys[-1]
        origin_yaw = yaws[-1]

    return xs, ys, yaws, directions


def interpolate(dist, length, mode, max_curvature, origin_x, origin_y,
                origin_yaw):
    if mode == "S":
        x = origin_x + dist / max_curvature * math.cos(origin_yaw)
        y = origin_y + dist / max_curvature * math.sin(origin_yaw)
        yaw = origin_yaw
    else:  # curve
        ldx = math.sin(dist) / max_curvature
        ldy = 0.0
        yaw = None
        if mode == "L":  # left turn
            ldy = (1.0 - math.cos(dist)) / max_curvature
            yaw = origin_yaw + dist
        elif mode == "R":  # right turn
            ldy = (1.0 - math.cos(dist)) / -max_curvature
            yaw = origin_yaw - dist
        gdx = math.cos(-origin_yaw) * ldx + math.sin(-origin_yaw) * ldy
        gdy = -math.sin(-origin_yaw) * ldx + math.cos(-origin_yaw) * ldy
        x = origin_x + gdx
        y = origin_y + gdy

    return x, y, yaw, 1 if length > 0.0 else -1


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size):
    q0 = [sx, sy, syaw]
    q1 = [gx, gy, gyaw]

    paths = generate_path(q0, q1, maxc, step_size)
    for path in paths:
        xs, ys, yaws, directions = generate_local_course(path.lengths,
                                                         path.ctypes, maxc,
                                                         step_size * maxc)

        # convert global coordinate
        path.x = [math.cos(-q0[2]) * ix + math.sin(-q0[2]) * iy + q0[0] for
                  (ix, iy) in zip(xs, ys)]
        path.y = [-math.sin(-q0[2]) * ix + math.cos(-q0[2]) * iy + q0[1] for
                  (ix, iy) in zip(xs, ys)]
        path.yaw = [pi_2_pi(yaw + q0[2]) for yaw in yaws]
        path.directions = directions
        path.lengths = [length / maxc for length in path.lengths]
        path.L = path.L / maxc

    return paths


def reeds_shepp_path_planning(sx, sy, syaw, gx, gy, gyaw, maxc, step_size=0.2):
    paths = calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size)
    if not paths:
        return None, None, None, None, None  # could not generate any path

    # search minimum cost path
    best_path_index = paths.index(min(paths, key=lambda p: abs(p.L)))
    b_path = paths[best_path_index]

    return b_path.x, b_path.y, b_path.yaw, b_path.ctypes, b_path.lengths


def main():
    print("Reeds Shepp path planner sample start!!")

    start_x = -1.0  # [m]
    start_y = -4.0  # [m]
    start_yaw = np.deg2rad(-20.0)  # [rad]

    end_x = 5.0  # [m]
    end_y = 5.0  # [m]
    end_yaw = np.deg2rad(25.0)  # [rad]

    curvature = 0.1
    step_size = 0.05

    xs, ys, yaws, modes, lengths = reeds_shepp_path_planning(start_x, start_y,
                                                             start_yaw, end_x,
                                                             end_y, end_yaw,
                                                             curvature,
                                                             step_size)

    if show_animation:  # pragma: no cover
        plt.cla()
        plt.plot(xs, ys, label="final course " + str(modes))
        print(f"{lengths=}")

        # plotting
        plot_arrow(start_x, start_y, start_yaw)
        plot_arrow(end_x, end_y, end_yaw)

        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()

    if not xs:
        assert False, "No path"


if __name__ == '__main__':
    main()
