"""

Potential Field based path planner


author: Atsushi Sakai (@Atsushi_twi)

"""

#  import numpy as np
import matplotlib.pyplot as plt


def main():

    sx = 0.0
    sy = 10.0
    gx = 30.0
    gy = 30.0

    plt.plot(sx, sy, "*r")
    plt.plot(gx, gy, "*r")
    plt.grid(True)
    plt.axis("equal")
    plt.show()

    print(__file__ + " start!!")


if __name__ == '__main__':
    main()
