
import matplotlib.pyplot as plt
import numpy as np


def main():
    print(__file__ + " start!!")

    ox, oy = create_dummy_observation_data()

    plt.plot(ox, oy, ".r")
    plt.axis("equal")
    plt.show()


def create_dummy_observation_data():
    ox = []
    oy = []
    # left corridor
    for y in range(-50, 50):
        ox.append(-20.0)
        oy.append(y)
    # right corridor 1
    for y in range(-50, 0):
        ox.append(20.0)
        oy.append(y)
    # right corridor 2
    for x in range(20, 50):
        ox.append(x)
        oy.append(0)
    # right corridor 3
    for x in range(20, 50):
        ox.append(x)
        oy.append(x/2.0+10)
    # right corridor 4
    for y in range(20, 50):
        ox.append(20)
        oy.append(y)
    ox = np.array(ox)
    oy = np.array(oy)
    # Adding random noize
    ox += np.random.rand(len(ox)) * 1.0
    oy += np.random.rand(len(ox)) * 1.0
    return ox, oy


if __name__ == '__main__':
    main()
