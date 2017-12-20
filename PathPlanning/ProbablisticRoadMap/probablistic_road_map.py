"""

Probablistic Road Map (PRM) Planner

author: Atsushi Sakai (@Atsushi_twi)

"""

import matplotlib.pyplot as plt
from matplotrecorder import matplotrecorder
matplotrecorder.donothing = True


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    robot_size = 1.0  # [m]

    ox = []
    oy = []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)

    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "xr")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")

    #  rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)

    #  plt.plot(rx, ry, "-r")

    for i in range(20):
        matplotrecorder.save_frame()
    plt.show()

    matplotrecorder.save_movie("animation.gif", 0.1)


if __name__ == '__main__':
    main()
