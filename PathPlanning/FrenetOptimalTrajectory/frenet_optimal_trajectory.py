"""



author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import matplotlib.pyplot as plt


class quinic_polynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):

        # calc coefficient of quinic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[T**3, T**4, T**5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T**2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3

        return xt


def frenet_optimal_planning():

    for di in np.arange(-5.0, 5.0, 1.0):
        for Ti in np.arange(1.0, 10.0, 1.0):
            lat_qp = quinic_polynomial(0.0, 0.0, 0.0, di, 0.0, 0.0, Ti)

            time = []
            d = []

            for t in np.arange(0.0, Ti, 0.1):
                time.append(t)
                d.append(lat_qp.calc_point(t))

            plt.plot(time, d)


def main():
    print(__file__ + " start!!")

    frenet_optimal_planning()

    plt.show()


if __name__ == '__main__':
    main()
