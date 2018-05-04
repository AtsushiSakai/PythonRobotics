"""

LQR local path planning module

author: Atsushi Sakai (@Atsushi_twi)

"""

import matplotlib.pyplot as plt

show_animation = True


def LQRplanning(sx, sy, gx, gy):

    rx, ry = [], []

    return rx, ry


def main():
    print(__file__ + " start!!")

    sx = 0.0
    sy = 0.0
    gx = 10.0
    gy = 5.0

    rx, ry = LQRplanning(sx, sy, gx, gy)

    plt.plot(sx, sy, "xb")
    plt.plot(gx, gy, "xb")
    plt.plot(rx, ry)
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
