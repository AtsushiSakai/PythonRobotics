"""

Reeds Shepp path planner sample code

author Atsushi Sakai(@Atsushi_twi)

"""
import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class Path:

    def __init__(self):
        self.lengths = []
        self.ctypes = []
        self.L = 0.0
        self.x = []
        self.y = []
        self.yaw = []
        self.directions = []


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def mod2pi(x):
    v = np.mod(x, 2.0 * np.pi)
    if v < -np.pi:
        v += 2.0 * np.pi
    else:
        if v > np.pi:
            v -= 2.0 * np.pi
    return v


def SLS(x, y, phi):
    phi = mod2pi(phi)
    if y > 0.0 and phi > 0.0 and phi < np.pi * 0.99:
        xd = - y / np.tan(phi) + x
        t = xd - np.tan(phi / 2.0)
        u = phi
        v = np.hypot(x - xd, y) - np.tan(phi / 2.0)
        return True, t, u, v
    elif y < 0.0 and phi > 0.0 and phi < np.pi * 0.99:
        xd = - y / np.tan(phi) + x
        t = xd - np.tan(phi / 2.0)
        u = phi
        v = -np.hypot(x - xd, y) - np.tan(phi / 2.0)
        return True, t, u, v

    return False, 0.0, 0.0, 0.0


def set_path(paths, lengths, ctypes):

    path = Path()
    path.ctypes = ctypes
    path.lengths = lengths

    # check same path exist
    for tpath in paths:
        typeissame = (tpath.ctypes == path.ctypes)
        if typeissame:
            if sum(tpath.lengths) - sum(path.lengths) <= 0.01:
                return paths  # not insert path

    path.L = sum([abs(i) for i in lengths])

    # Base.Test.@test path.L >= 0.01
    if path.L >= 0.01:
        paths.append(path)

    return paths


def SCS(x, y, phi, paths):
    flag, t, u, v = SLS(x, y, phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["S", "L", "S"])

    flag, t, u, v = SLS(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["S", "R", "S"])

    return paths


def polar(x, y):
    r = np.hypot(x, y)
    theta = np.arctan2(y, x)
    return r, theta


def LSL(x, y, phi):
    u, t = polar(x - np.sin(phi), y - 1.0 + np.cos(phi))
    if t >= 0.0:
        v = mod2pi(phi - t)
        if v >= 0.0:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0


def LRL(x, y, phi):
    u1, t1 = polar(x - np.sin(phi), y - 1.0 + np.cos(phi))

    if u1 <= 4.0:
        u = -2.0 * np.arcsin(0.25 * u1)
        t = mod2pi(t1 + 0.5 * u + np.pi)
        v = mod2pi(phi - t + u)

        if t >= 0.0 and u <= 0.0:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0


def CCC(x, y, phi, paths):

    flag, t, u, v = LRL(x, y, phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["L", "R", "L"])

    flag, t, u, v = LRL(-x, y, -phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["L", "R", "L"])

    flag, t, u, v = LRL(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["R", "L", "R"])

    flag, t, u, v = LRL(-x, -y, phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R", "L", "R"])

    # backwards
    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)
    xb = x * cos_phi + y * sin_phi
    yb = x * sin_phi - y * cos_phi
    # println(xb, ",", yb,",",x,",",y)

    flag, t, u, v = LRL(xb, yb, phi)
    if flag:
        paths = set_path(paths, [v, u, t], ["L", "R", "L"])

    flag, t, u, v = LRL(-xb, yb, -phi)
    if flag:
        paths = set_path(paths, [-v, -u, -t], ["L", "R", "L"])

    flag, t, u, v = LRL(xb, -yb, -phi)
    if flag:
        paths = set_path(paths, [v, u, t], ["R", "L", "R"])

    flag, t, u, v = LRL(-xb, -yb, phi)
    if flag:
        paths = set_path(paths, [-v, -u, -t], ["R", "L", "R"])

    return paths


def CSC(x, y, phi, paths):
    flag, t, u, v = LSL(x, y, phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["L", "S", "L"])

    flag, t, u, v = LSL(-x, y, -phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["L", "S", "L"])

    flag, t, u, v = LSL(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["R", "S", "R"])

    flag, t, u, v = LSL(-x, -y, phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R", "S", "R"])

    flag, t, u, v = LSR(x, y, phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["L", "S", "R"])

    flag, t, u, v = LSR(-x, y, -phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["L", "S", "R"])

    flag, t, u, v = LSR(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["R", "S", "L"])

    flag, t, u, v = LSR(-x, -y, phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R", "S", "L"])

    return paths


def LSR(x, y, phi):
    u1, t1 = polar(x + np.sin(phi), y - 1.0 - np.cos(phi))
    u1 = u1 ** 2
    if u1 >= 4.0:
        u = np.sqrt(u1 - 4.0)
        theta = np.arctan2(2.0, u)
        t = mod2pi(t1 + theta)
        v = mod2pi(t - phi)

        if t >= 0.0 and v >= 0.0:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0


def generate_path(q0, q1, maxc):
    dx = q1[0] - q0[0]
    dy = q1[1] - q0[1]
    dth = q1[2] - q0[2]
    c = np.cos(q0[2])
    s = np.sin(q0[2])
    x = (c * dx + s * dy) * maxc
    y = (-s * dx + c * dy) * maxc

    paths = []
    paths = SCS(x, y, dth, paths)
    paths = CSC(x, y, dth, paths)
    paths = CCC(x, y, dth, paths)

    return paths


def interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions):

    if m == "S":
        px[ind] = ox + l / maxc * np.cos(oyaw)
        py[ind] = oy + l / maxc * np.sin(oyaw)
        pyaw[ind] = oyaw
    else:  # curve
        ldx = np.sin(l) / maxc
        if m == "L":  # left turn
            ldy = (1.0 - np.cos(l)) / maxc
        elif m == "R":  # right turn
            ldy = (1.0 - np.cos(l)) / -maxc
        gdx = np.cos(-oyaw) * ldx + np.sin(-oyaw) * ldy
        gdy = -np.sin(-oyaw) * ldx + np.cos(-oyaw) * ldy
        px[ind] = ox + gdx
        py[ind] = oy + gdy

    if m == "L":  # left turn
        pyaw[ind] = oyaw + l
    elif m == "R":  # right turn
        pyaw[ind] = oyaw - l

    if l > 0.0:
        directions[ind] = 1
    else:
        directions[ind] = -1

    return px, py, pyaw, directions


def generate_local_course(L, lengths, mode, maxc, step_size):
    npoint = int(np.floor(L / step_size)) + len(lengths) + 4

    px = [0.0 for i in range(npoint)]
    py = [0.0 for i in range(npoint)]
    pyaw = [0.0 for i in range(npoint)]
    directions = [0.0 for i in range(npoint)]
    ind = 1

    if lengths[0] > 0.0:
        directions[0] = 1
    else:
        directions[0] = -1

    if lengths[0] > 0.0:
        d = step_size
    else:
        d = -step_size

    pd = d
    ll = 0.0

    for (m, l, i) in zip(mode, lengths, range(len(mode))):
        if l > 0.0:
            d = step_size
        else:
            d = -step_size

        # set origin state
        ox, oy, oyaw = px[ind], py[ind], pyaw[ind]

        ind -= 1
        if i >= 1 and (lengths[i - 1] * lengths[i]) > 0:
            pd = - d - ll
        else:
            pd = d - ll

        while abs(pd) <= abs(l):
            ind += 1
            px, py, pyaw, directions = interpolate(
                ind, pd, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)
            pd += d

        ll = l - pd - d  # calc remain length

        ind += 1
        px, py, pyaw, directions = interpolate(
            ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)

    # remove unused data
    while px[-1] == 0.0:
        px.pop()
        py.pop()
        pyaw.pop()
        directions.pop()

    return px, py, pyaw, directions


def pi_2_pi(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi


def calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size):
    q0 = [sx, sy, syaw]
    q1 = [gx, gy, gyaw]

    paths = generate_path(q0, q1, maxc)
    for path in paths:
        x, y, yaw, directions = generate_local_course(
            path.L, path.lengths, path.ctypes, maxc, step_size * maxc)

        # convert global coordinate
        path.x = [np.cos(-q0[2]) * ix + np.sin(-q0[2])
                  * iy + q0[0] for (ix, iy) in zip(x, y)]
        path.y = [-np.sin(-q0[2]) * ix + np.cos(-q0[2])
                  * iy + q0[1] for (ix, iy) in zip(x, y)]
        path.yaw = [pi_2_pi(iyaw + q0[2]) for iyaw in yaw]
        path.directions = directions
        path.lengths = [l / maxc for l in path.lengths]
        path.L = path.L / maxc

    #  print(paths)

    return paths


def reeds_shepp_path_planning(sx, sy, syaw,
                              gx, gy, gyaw, maxc, step_size):

    paths = calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size)

    if len(paths) == 0:
        #  print("No path")
        #  print(sx, sy, syaw, gx, gy, gyaw)
        return None, None, None, None, None

    minL = np.inf
    best_path_index = -1
    for i in range(len(paths)):
        if paths[i].L <= minL:
            minL = paths[i].L
            best_path_index = i

    bpath = paths[best_path_index]

    return bpath.x, bpath.y, bpath.yaw, bpath.ctypes, bpath.lengths


def test():

    NTEST = 5

    for _ in range(NTEST):
        start_x = (np.random.rand() - 0.5) * 10.0  # [m]
        start_y = (np.random.rand() - 0.5) * 10.0  # [m]
        start_yaw = np.deg2rad((np.random.rand() - 0.5) * 180.0)  # [rad]

        end_x = (np.random.rand() - 0.5) * 10.0  # [m]
        end_y = (np.random.rand() - 0.5) * 10.0  # [m]
        end_yaw = np.deg2rad((np.random.rand() - 0.5) * 180.0)  # [rad]

        curvature = 1.0 / (np.random.rand() * 20.0)
        step_size = 0.1

        px, py, _, mode, _ = reeds_shepp_path_planning(
            start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature, step_size)

        if show_animation:
            plt.cla()
            plt.plot(px, py, label="final course {}".format(mode))

            #  plotting
            plot_arrow(start_x, start_y, start_yaw)
            plot_arrow(end_x, end_y, end_yaw)

            plt.legend()
            plt.grid(True)
            plt.axis("equal")
            plt.xlim(-10, 10)
            plt.ylim(-10, 10)
            plt.pause(1.0)

            #  plt.show()

    print("Test done")


def main():
    print("Reeds Shepp path planner sample start!!")

    start_x = -1.0  # [m]
    start_y = -4.0  # [m]
    start_yaw = np.deg2rad(-20.0)  # [rad]

    end_x = 5.0  # [m]
    end_y = 5.0  # [m]
    end_yaw = np.deg2rad(25.0)  # [rad]

    curvature = 1.0
    step_size = 0.1

    px, py, _, mode, _ = reeds_shepp_path_planning(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature, step_size)

    if show_animation:
        plt.cla()
        plt.plot(px, py, label="final course {}".format(mode))

        # plotting
        plot_arrow(start_x, start_y, start_yaw)
        plot_arrow(end_x, end_y, end_yaw)

        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()

    if not px:
        assert False, "No path"


if __name__ == '__main__':
    test()
    main()
