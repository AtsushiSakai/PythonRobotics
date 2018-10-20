"""

Dubins path planner sample code

author Atsushi Sakai (@Atsushi_twi)

"""
import matplotlib.pyplot as plt
import numpy as np


def mod2pi(theta):
    return theta - 2.0 * np.pi * np.floor(theta / 2.0 / np.pi)


def pi_2_pi(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi


def LSL(alpha, beta, d):
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)
    c_ab = np.cos(alpha - beta)

    tmp0 = d + sa - sb

    mode = ["L", "S", "L"]
    p_squared = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb)
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = np.arctan2(cb - ca, tmp0)
    t = mod2pi(-alpha + tmp1)
    p = np.sqrt(p_squared)
    q = mod2pi(beta - tmp1)
    #  print(math.degrees(t), p, math.degrees(q))

    return t, p, q, mode


def RSR(alpha, beta, d):
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)
    c_ab = np.cos(alpha - beta)

    tmp0 = d - sa + sb
    mode = ["R", "S", "R"]
    p_squared = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa)
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = np.arctan2(ca - cb, tmp0)
    t = mod2pi(alpha - tmp1)
    p = np.sqrt(p_squared)
    q = mod2pi(-beta + tmp1)

    return t, p, q, mode


def LSR(alpha, beta, d):
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)
    c_ab = np.cos(alpha - beta)

    p_squared = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb)
    mode = ["L", "S", "R"]
    if p_squared < 0:
        return None, None, None, mode
    p = np.sqrt(p_squared)
    tmp2 = np.arctan2(-ca - cb, d + sa + sb) - np.arctan2(-2.0, p)
    t = mod2pi(-alpha + tmp2)
    q = mod2pi(-mod2pi(beta) + tmp2)

    return t, p, q, mode


def RSL(alpha, beta, d):
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)
    c_ab = np.cos(alpha - beta)

    p_squared = d * d - 2 + 2 * c_ab - 2 * d * (sa + sb)
    mode = ["R", "S", "L"]
    if p_squared < 0:
        return None, None, None, mode
    p = np.sqrt(p_squared)
    tmp2 = np.arctan2(ca + cb, d - sa - sb) - np.arctan2(2.0, p)
    t = mod2pi(alpha - tmp2)
    q = mod2pi(beta - tmp2)

    return t, p, q, mode


def RLR(alpha, beta, d):
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)
    c_ab = np.cos(alpha - beta)

    mode = ["R", "L", "R"]
    tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
    if abs(tmp_rlr) > 1.0:
        return None, None, None, mode

    p = mod2pi(2 * np.pi - np.arccos(tmp_rlr))
    t = mod2pi(alpha - np.arctan2(ca - cb, d - sa + sb) + mod2pi(p / 2.0))
    q = mod2pi(alpha - beta - t + mod2pi(p))
    return t, p, q, mode


def LRL(alpha, beta, d):
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)
    c_ab = np.cos(alpha - beta)

    mode = ["L", "R", "L"]
    tmp_lrl = (6.0 - d * d + 2 * c_ab + 2 * d * (- sa + sb)) / 8.0
    if abs(tmp_lrl) > 1:
        return None, None, None, mode
    p = mod2pi(2 * np.pi - np.arccos(tmp_lrl))
    t = mod2pi(-alpha - np.arctan2(ca - cb, d + sa - sb) + p / 2.0)
    q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p))

    return t, p, q, mode


def dubins_path_planning_from_origin(ex, ey, eyaw, c):
    # nomalize
    dx = ex
    dy = ey
    D = np.hypot(dx, dy)
    d = D / c
    #  print(dx, dy, D, d)

    theta = mod2pi(np.arctan2(dy, dx))
    alpha = mod2pi(-theta)
    beta = mod2pi(eyaw - theta)
    #  print(theta, alpha, beta, d)

    planners = [LSL, RSR, LSR, RSL, RLR, LRL]

    bcost = np.inf
    bt, bp, bq, bmode = None, None, None, None

    for planner in planners:
        t, p, q, mode = planner(alpha, beta, d)
        if t is None:
            #  print("".join(mode) + " cannot generate path")
            continue

        cost = abs(t) + abs(p) + abs(q)
        if bcost > cost:
            bt, bp, bq, bmode = t, p, q, mode
            bcost = cost

    #  print(bmode)
    px, py, pyaw = generate_course([bt, bp, bq], bmode, c)

    return px, py, pyaw, bmode, bcost


def dubins_path_planning(sx, sy, syaw, ex, ey, eyaw, c):
    """
    Dubins path plannner

    input:
        sx x position of start point [m]
        sy y position of start point [m]
        syaw yaw angle of start point [rad]
        ex x position of end point [m]
        ey y position of end point [m]
        eyaw yaw angle of end point [rad]
        c curvature [1/m]

    output:
        px
        py
        pyaw
        mode

    """

    ex = ex - sx
    ey = ey - sy

    sin_syaw = np.sin(syaw)
    cos_syaw = np.cos(syaw)

    lex = cos_syaw * ex + sin_syaw * ey
    ley = -sin_syaw * ex + cos_syaw * ey
    leyaw = eyaw - syaw

    lpx, lpy, lpyaw, mode, clen = dubins_path_planning_from_origin(
        lex, ley, leyaw, c)

    px = [cos_syaw * x - sin_syaw * y + sx for x, y in zip(lpx, lpy)]
    py = [sin_syaw * x + cos_syaw * y + sy for x, y in zip(lpx, lpy)]
    pyaw = [pi_2_pi(iyaw + syaw) for iyaw in lpyaw]
    #  print(syaw)
    #  pyaw = lpyaw

    #  plt.plot(pyaw, "-r")
    #  plt.plot(lpyaw, "-b")
    #  plt.plot(eyaw, "*r")
    #  plt.plot(syaw, "*b")
    #  plt.show()

    return px, py, pyaw, mode, clen


def generate_course(length, mode, c):

    px = [0.0]
    py = [0.0]
    pyaw = [0.0]

    for m, l in zip(mode, length):
        pd = 0.0
        if m is "S":
            d = 1.0 / c
        else:  # turning couse
            d = np.deg2rad(3.0)

        while pd < abs(l - d):
            #  print(pd, l)
            px.append(px[-1] + d * c * np.cos(pyaw[-1]))
            py.append(py[-1] + d * c * np.sin(pyaw[-1]))

            if m is "L":  # left turn
                pyaw.append(pyaw[-1] + d)
            elif m is "S":  # Straight
                pyaw.append(pyaw[-1])
            elif m is "R":  # right turn
                pyaw.append(pyaw[-1] - d)
            pd += d

        d = l - pd
        px.append(px[-1] + d * c * np.cos(pyaw[-1]))
        py.append(py[-1] + d * c * np.sin(pyaw[-1]))

        if m is "L":  # left turn
            pyaw.append(pyaw[-1] + d)
        elif m is "S":  # Straight
            pyaw.append(pyaw[-1])
        elif m is "R":  # right turn
            pyaw.append(pyaw[-1] - d)
        pd += d

    return px, py, pyaw


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


if __name__ == '__main__':
    print("Dubins path planner sample start!!")

    start_x = 1.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = np.deg2rad(45.0)  # [rad]

    end_x = -3.0  # [m]
    end_y = -3.0  # [m]
    end_yaw = np.deg2rad(-45.0)  # [rad]

    curvature = 1.0

    px, py, pyaw, mode, clen = dubins_path_planning(start_x, start_y,
                                                    start_yaw, end_x, end_y,
                                                    end_yaw, curvature)

    plt.plot(px, py, label="final course {}".format("".join(mode)))

    # plotting
    plot_arrow(start_x, start_y, start_yaw)
    plot_arrow(end_x, end_y, end_yaw)

    #  for (ix, iy, iyaw) in zip(px, py, pyaw):
    #  plot_arrow(ix, iy, iyaw, fc="b")

    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()
