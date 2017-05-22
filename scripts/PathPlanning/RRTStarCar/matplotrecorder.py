"""
 A simple Python module for recording matplotlib animation

 This tool use convert command of ImageMagick

 author: Atsushi Sakai
"""

import matplotlib.pyplot as plt
import subprocess

iframe = 0
donothing = False


def save_frame():
    """
    Save a frame for movie
    """

    if not donothing:
        global iframe
        plt.savefig("recoder" + '{0:04d}'.format(iframe) + '.png')
        iframe += 1


def save_movie(fname, d_pause):
    """
    Save movie as gif
    """
    if not donothing:
        cmd = "convert -delay " + str(int(d_pause * 100)) + \
            " recoder*.png " + fname
        subprocess.call(cmd, shell=True)
        cmd = "rm recoder*.png"
        subprocess.call(cmd, shell=True)


if __name__ == '__main__':
    print("A sample recording start")
    import math

    time = range(50)

    x1 = [math.cos(t / 10.0) for t in time]
    y1 = [math.sin(t / 10.0) for t in time]
    x2 = [math.cos(t / 10.0) + 2 for t in time]
    y2 = [math.sin(t / 10.0) + 2 for t in time]

    for ix1, iy1, ix2, iy2 in zip(x1, y1, x2, y2):
        plt.plot(ix1, iy1, "xr")
        plt.plot(ix2, iy2, "xb")
        plt.axis("equal")
        plt.pause(0.1)

        save_frame()  # save each frame

    save_movie("animation.gif", 0.1)
    #  save_movie("animation.mp4", 0.1)
