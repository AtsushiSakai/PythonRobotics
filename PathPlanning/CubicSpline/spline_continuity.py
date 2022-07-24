
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate


class Spline2D:

    def __init__(self, x, y, kind="cubic"):
        self.s = self.__calc_s(x, y)
        self.sx = interpolate.interp1d(self.s, x, kind=kind)
        self.sy = interpolate.interp1d(self.s, y, kind=kind)

    def __calc_s(self, x, y):
        self.ds = np.hypot(np.diff(x), np.diff(y))
        s = [0.0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        x = self.sx(s)
        y = self.sy(s)
        return x, y


def main():
    x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    y = [0.7, -6, -5, -3.5, 0.0, 5.0, -2.0]
    ds = 0.1  # [m] distance of each interpolated points

    plt.subplots(1)
    plt.plot(x, y, "xb", label="Data points")

    for (kind, label) in [("linear", "C0 (Linear spline)"),
                          ("quadratic", "C0 & C1 (Quadratic spline)"),
                          ("cubic", "C0 & C1 & C2 (Cubic spline)")]:
        rx, ry = [], []
        sp = Spline2D(x, y, kind=kind)
        s = np.arange(0, sp.s[-1], ds)
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
        plt.plot(rx, ry, "-", label=label)

    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
