"""

Object clustering with k-mean algorithm


author: Atsushi Sakai (@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import random


def calc_raw_data():

    rx, ry = [], []

    cx = [0.0, 5.0]
    cy = [0.0, 5.0]
    np = 30
    rand_d = 3.0

    for (icx, icy) in zip(cx, cy):
        for _ in range(np):
            rx.append(icx + rand_d * (random.random() - 0.5))
            ry.append(icy + rand_d * (random.random() - 0.5))

    return rx, ry


def main():
    print(__file__ + " start!!")

    rx, ry = calc_raw_data()

    plt.plot(rx, ry, "x")
    plt.show()


if __name__ == '__main__':
    main()
