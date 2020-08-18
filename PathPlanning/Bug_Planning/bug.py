"""
Bug Planning
author: Sarim Mehdi(muhammadsarim.mehdi@studio.unibo.it)
Source: https://sites.google.com/site/ece452bugalgorithms/
"""

import numpy as np
import matplotlib.pyplot as plt


class Bug():
    def __init__(self, startx, starty, goalx, goaly, obsx, obsy):
        self.goalx = goalx
        self.goaly = goaly
        self.obsx = obsx
        self.obsy = obsy
        self.rx = [startx]
        self.ry = [starty]
        self.outx = []
        self.outy = []
        for o_x, o_y in zip(obsx, obsy):
            for addx, addy in zip([1, 0, -1, -1, -1, 0, 1, 1],
                                  [1, 1, 1, 0, -1, -1, -1, 0]):
                candx, candy = o_x+addx, o_y+addy
                valid_point = True
                for _x, _y in zip(obsx, obsy):
                    if candx == _x and candy == _y:
                        valid_point = False
                        break
                if valid_point:
                    self.outx.append(candx), self.outy.append(candy)

    def mov_normal(self):
        return self.rx[-1] + np.sign(self.goalx - self.rx[-1]), \
               self.ry[-1] + np.sign(self.goaly - self.ry[-1])

    def mov_to_next_obs(self, visitedx, visitedy):
        for addx, addy in zip([1, 0, -1, 0], [0, 1, 0, -1]):
            cx, cy = self.rx[-1] + addx, self.ry[-1] + addy
            for _x, _y in zip(self.outx, self.outy):
                use_pt = True
                if cx == _x and cy == _y:
                    for v_x, v_y in zip(visitedx, visitedy):
                        if cx == v_x and cy == v_y:
                            use_pt = False
                            break
                    if use_pt:
                        return cx, cy, False
                if not use_pt:
                    break
        return self.rx[-1], self.ry[-1], True

    def bug0(self):
        mov_dir = 'normal'
        candx, candy = -9999, -9999
        plt.plot(self.obsx, self.obsy, ".k")
        plt.plot(self.rx[-1], self.ry[-1], "og")
        plt.plot(self.goalx, self.goaly, "xb")
        plt.plot(self.outx, self.outy, ".")
        plt.grid(True)
        plt.title('BUG 0')

        for xob, yob in zip(self.outx, self.outy):
            if self.rx[-1] == xob and self.ry[-1] == yob:
                mov_dir = 'obs'
                break

        visitedx, visitedy = [], []
        while True:
            if self.rx[-1] == self.goalx and \
                    self.ry[-1] == self.goaly:
                break
            if mov_dir == 'normal':
                candx, candy = self.mov_normal()
            if mov_dir == 'obs':
                candx, candy, _ = self.mov_to_next_obs(visitedx, visitedy)
            if mov_dir == 'normal':
                found_boundary = False
                for xob, yob in zip(self.outx, self.outy):
                    if candx == xob and candy == yob:
                        self.rx.append(candx), self.ry.append(candy)
                        visitedx[:], visitedy[:] = [], []
                        visitedx.append(candx), visitedy.append(candy)
                        mov_dir = 'obs'
                        found_boundary = True
                        break
                if not found_boundary:
                    self.rx.append(candx), self.ry.append(candy)
            elif mov_dir == 'obs':
                can_go_normal = True
                for xob, yob in zip(self.obsx, self.obsy):
                    if self.mov_normal()[0] == xob and \
                            self.mov_normal()[1] == yob:
                        can_go_normal = False
                        break
                if can_go_normal:
                    mov_dir = 'normal'
                else:
                    self.rx.append(candx), self.ry.append(candy)
                    visitedx.append(candx), visitedy.append(candy)
            plt.plot(self.rx, self.ry, "-r")
            plt.pause(0.001)
        plt.show()

    def bug1(self):
        mov_dir = 'normal'
        candx, candy = -9999, -9999
        exitx, exity = -9999, -9999
        dist = 9999
        back_to_start = False
        second_round = False
        plt.plot(self.obsx, self.obsy, ".k")
        plt.plot(self.rx[-1], self.ry[-1], "og")
        plt.plot(self.goalx, self.goaly, "xb")
        plt.plot(self.outx, self.outy, ".")
        plt.grid(True)
        plt.title('BUG 1')

        for xob, yob in zip(self.outx, self.outy):
            if self.rx[-1] == xob and self.ry[-1] == yob:
                mov_dir = 'obs'
                break

        visitedx, visitedy = [], []
        while True:
            if self.rx[-1] == self.goalx and \
                    self.ry[-1] == self.goaly:
                break
            if mov_dir == 'normal':
                candx, candy = self.mov_normal()
            if mov_dir == 'obs':
                candx, candy, back_to_start = \
                    self.mov_to_next_obs(visitedx, visitedy)
            if mov_dir == 'normal':
                found_boundary = False
                for xob, yob in zip(self.outx, self.outy):
                    if candx == xob and candy == yob:
                        self.rx.append(candx), self.ry.append(candy)
                        visitedx[:], visitedy[:] = [], []
                        visitedx.append(candx), visitedy.append(candy)
                        mov_dir = 'obs'
                        dist = 9999
                        back_to_start = False
                        second_round = False
                        found_boundary = True
                        break
                if not found_boundary:
                    self.rx.append(candx), self.ry.append(candy)
            elif mov_dir == 'obs':
                d = np.linalg.norm(np.array([candx, candy] -
                                            np.array([self.goalx,
                                                      self.goaly])))
                if d < dist and not second_round:
                    exitx, exity = candx, candy
                    dist = d
                if back_to_start and not second_round:
                    second_round = True
                    del self.rx[-len(visitedx):]
                    del self.ry[-len(visitedy):]
                    visitedx[:], visitedy[:] = [], []
                self.rx.append(candx), self.ry.append(candy)
                visitedx.append(candx), visitedy.append(candy)
                if candx == exitx and \
                        candy == exity and \
                        second_round:
                    mov_dir = 'normal'
            plt.plot(self.rx, self.ry, "-r")
            plt.pause(0.001)
        plt.show()

    def bug2(self):
        mov_dir = 'normal'
        candx, candy = -9999, -9999
        plt.plot(self.obsx, self.obsy, ".k")
        plt.plot(self.rx[-1], self.ry[-1], "og")
        plt.plot(self.goalx, self.goaly, "xb")
        plt.plot(self.outx, self.outy, ".")

        straightx, straighty = [self.rx[-1]], [self.ry[-1]]
        hitx, hity = [], []
        while True:
            if straightx[-1] == self.goalx and \
                    straighty[-1] == self.goaly:
                break
            c_x = straightx[-1] + np.sign(self.goalx - straightx[-1])
            c_y = straighty[-1] + np.sign(self.goaly - straighty[-1])
            for xob, yob in zip(self.outx, self.outy):
                if c_x == xob and c_y == yob:
                    hitx.append(c_x), hity.append(c_y)
                    break
            straightx.append(c_x), straighty.append(c_y)
        plt.plot(straightx, straighty, ",")
        plt.plot(hitx, hity, "d")

        plt.grid(True)
        plt.title('BUG 2')

        for xob, yob in zip(self.outx, self.outy):
            if self.rx[-1] == xob and self.ry[-1] == yob:
                mov_dir = 'obs'
                break

        visitedx, visitedy = [], []
        while True:
            if self.rx[-1] == self.goalx \
                    and self.ry[-1] == self.goaly:
                break
            if mov_dir == 'normal':
                candx, candy = self.mov_normal()
            if mov_dir == 'obs':
                candx, candy, _ = self.mov_to_next_obs(visitedx, visitedy)
            if mov_dir == 'normal':
                found_boundary = False
                for xob, yob in zip(self.outx, self.outy):
                    if candx == xob and candy == yob:
                        self.rx.append(candx), self.ry.append(candy)
                        visitedx[:], visitedy[:] = [], []
                        visitedx.append(candx), visitedy.append(candy)
                        del hitx[0]
                        del hity[0]
                        mov_dir = 'obs'
                        found_boundary = True
                        break
                if not found_boundary:
                    self.rx.append(candx), self.ry.append(candy)
            elif mov_dir == 'obs':
                self.rx.append(candx), self.ry.append(candy)
                visitedx.append(candx), visitedy.append(candy)
                for i_x, i_y in zip(range(len(hitx)), range(len(hity))):
                    if candx == hitx[i_x] and candy == hity[i_y]:
                        del hitx[i_x]
                        del hity[i_y]
                        mov_dir = 'normal'
                        break
            plt.plot(self.rx, self.ry, "-r")
            plt.pause(0.001)
        plt.show()


def main():
    # set obstacle positions
    ox, oy = [], []

    sx = 0.0
    sy = 0.0
    gx = 167.0
    gy = 50.0

    for i in range(20, 40):
        for j in range(20, 40):
            ox.append(i)
            oy.append(j)

    for i in range(60, 100):
        for j in range(40, 80):
            ox.append(i)
            oy.append(j)

    for i in range(120, 140):
        for j in range(80, 100):
            ox.append(i)
            oy.append(j)

    for i in range(80, 140):
        for j in range(0, 20):
            ox.append(i)
            oy.append(j)

    for i in range(0, 20):
        for j in range(60, 100):
            ox.append(i)
            oy.append(j)

    for i in range(20, 40):
        for j in range(80, 100):
            ox.append(i)
            oy.append(j)

    for i in range(120, 160):
        for j in range(40, 60):
            ox.append(i)
            oy.append(j)

    myBug = Bug(sx, sy, gx, gy, ox, oy)
    # myBug.bug0()

    # myBug.bug1()
    # showx, showy = [], []
    # for r_x, r_y in zip(myBug.rx, myBug.ry):
    #     showx.append(r_x), showy.append(r_y)
    #     plt.plot(showx, showy, "-r")
    #     plt.pause(0.001)
    # plt.show()

    myBug.bug2()


if __name__ == '__main__':
    main()
