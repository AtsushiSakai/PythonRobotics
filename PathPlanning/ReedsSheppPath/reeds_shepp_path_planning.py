"""

Reeds Shepp path planner sample code

author Atsushi Sakai(@Atsushi_twi)

"""
import reeds_shepp
import numpy as np
import math
import matplotlib.pyplot as plt

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
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def mod2pi(x):
    v = np.mod(x, 2.0 * math.pi)
    if v < -math.pi:
        v += 2.0 * math.pi
    else:
        if v > math.pi:
            v -= 2.0 * math.pi
    return v


def SLS(x, y, phi):
    # println(x,",", y,",", phi, ",", mod2pi(phi))
    phi = mod2pi(phi)
    if y > 0.0 and phi > 0.0 and phi < math.pi * 0.99:
        xd = - y / math.tan(phi) + x
        t = xd - math.tan(phi / 2.0)
        u = phi
        v = math.sqrt((x - xd) ** 2 + y ** 2) - math.tan(phi / 2.0)
        # println("1,",t,",",u,",",v)
        return True, t, u, v
    elif y < 0.0 and phi > 0.0 and phi < math.pi * 0.99:
        xd = - y / math.tan(phi) + x
        t = xd - math.tan(phi / 2.0)
        u = phi
        v = -math.sqrt((x - xd) ^ 2 + y ^ 2) - math.tan(phi / 2.0)
        # println("2,",t,",",u,",",v)
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
            if sum(tpath.lengths - path.lengths) <= 0.01:
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


def generate_path(q0, q1, maxc):
    dx = q1[0] - q0[0]
    dy = q1[1] - q0[1]
    dth = q1[2] - q0[2]
    c = math.cos(q0[2])
    s = math.sin(q0[2])
    x = (c * dx + s * dy) * maxc
    y = (-s * dx + c * dy) * maxc

    paths = []
    paths = SCS(x, y, dth, paths)
    #  paths = CSC(x, y, dth, paths)
    #  paths = CCC(x, y, dth, paths)

    return paths


#  def generate_local_course(L: : Float64,
    #  lengths: : Array{Float64},
    #  mode: : Array{String},
    #  maxc: : Float64,
    #  step_size: : Float64)
    #  npoint = trunc(Int64, L / step_size) + length(lengths) + 3
    #  # println(npoint, ",", L, ",", step_size, ",", L/step_size)

    #  px = fill(0.0, npoint)
    #  py = fill(0.0, npoint)
    #  pyaw = fill(0.0, npoint)
    #  directions = fill(0, npoint)
    #  ind = 2

    #  if lengths[1] > 0.0
    #  directions[1] = 1
    #  else
    #  directions[1] = -1
    #  end

    #  if lengths[1] > 0.0
    #  d = step_size
    #  else
    #  d = -step_size
    #  end

    #  pd = d
    #  ll = 0.0

    #  for (m, l, i) in zip(mode, lengths, 1: length(mode))

    #  if l > 0.0
    #  d = step_size
    #  else
    #  d = -step_size
    #  end

    #  # set prigin state
    #  ox, oy, oyaw = px[ind], py[ind], pyaw[ind]

    #  ind -= 1
    #  if i >= 2 & & (lengths[i - 1] * lengths[i]) > 0
    #  pd = - d - ll
    #  else
    #  pd = d - ll
    #  end

    #  while abs(pd) <= abs(l)
    #  ind += 1
    #  px, py, pyaw, directions = interpolate(
    #  ind, pd, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)
    #  pd += d
    #  end

    #  ll = l - pd - d  # calc remain length

    #  ind += 1
    #  px, py, pyaw, directions = interpolate(
    #  ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)
    #  end

    #  # remove unused data
    #  while px[end] == 0.0
    #  pop!(px)
    #  pop!(py)
    #  pop!(pyaw)
    #  pop!(directions)
    #  end

    #  return px, py, pyaw, directions


def calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size):
    q0 = [sx, sy, syaw]
    q1 = [gx, gy, gyaw]

    paths = generate_path(q0, q1, maxc)
    for path in paths:
        #  x, y, yaw, directions = generate_local_course(
        #  path.L, path.lengths, path.ctypes, maxc, step_size * maxc)
        pass

    #  # convert global coordinate
    #  path.x = [cos(-q0[3]) * ix + sin(-q0[3]) * iy + q0[1] for (ix, iy) in zip(x, y)]
    #  path.y = [-sin(-q0[3]) * ix + cos(-q0[3]) * iy + q0[2] for (ix, iy) in zip(x, y)]
    #  path.yaw = common_func.pi_2_pi.([iyaw + q0[3] for iyaw in yaw])
    #  path.directions = directions
    #  path.lengths = [l/maxc for l in path.lengths]
    #  path.L = path.L/maxc

    return paths


def reeds_shepp_path_planning2(sx, sy, syaw,
                               gx, gy, gyaw, maxc, step_size):

    paths = calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size)

    minL = float("Inf")
    best_path_index = -1
    for i in range(len(paths)):
        if paths[i].L <= minL:
            minL = paths[i].L
            best_path_index = i

    bpath = paths[best_path_index]

    xs = bpath.x
    ys = bpath.y
    yaw = bpath.yaw
    ptype = bpath.ctypes
    clen = bpath.lengths
    return xs, ys, yaw, ptype, clen


def reeds_shepp_path_planning(start_x, start_y, start_yaw,
                              end_x, end_y, end_yaw, curvature):
    step_size = 0.1
    q0 = [start_x, start_y, start_yaw]
    q1 = [end_x, end_y, end_yaw]
    qs = reeds_shepp.path_sample(q0, q1, 1.0 / curvature, step_size)
    xs = [q[0] for q in qs]
    ys = [q[1] for q in qs]
    yaw = [q[2] for q in qs]

    xs.append(end_x)
    ys.append(end_y)
    yaw.append(end_yaw)

    clen = reeds_shepp.path_length(q0, q1, 1.0 / curvature)
    pathtypeTuple = reeds_shepp.path_type(q0, q1, 1.0 / curvature)

    ptype = ""
    for t in pathtypeTuple:
        if t == 1:
            ptype += "L"
        elif t == 2:
            ptype += "S"
        elif t == 3:
            ptype += "R"

    return xs, ys, yaw, ptype, clen


def main():
    print("Reeds Shepp path planner sample start!!")

    start_x = 1.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = math.radians(0.0)  # [rad]

    end_x = 5.0  # [m]
    end_y = 10.0  # [m]
    end_yaw = math.radians(45.0)  # [rad]

    curvature = 1.0
    step_size = 0.1

    px, py, pyaw, mode, clen = reeds_shepp_path_planning2(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature, step_size)

    #  px, py, pyaw, mode, clen = reeds_shepp_path_planning(
    #  start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)

    if show_animation:
        plt.plot(px, py, label="final course " + str(mode))

        # plotting
        plot_arrow(start_x, start_y, start_yaw)
        plot_arrow(end_x, end_y, end_yaw)

        for (ix, iy, iyaw) in zip(px, py, pyaw):
            plot_arrow(ix, iy, iyaw, fc="b")
        #  print(clen)

        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()


if __name__ == '__main__':
    main()
