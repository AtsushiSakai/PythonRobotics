#!/usr/bin/env python
"""
 Pure pursuit controller V2

 Copyright (c) 2021 AIGRO B.V. - Jev Kuznetsov
"""

import time
import math
import turtle
import numpy as np
from PythonRobotics.vectors import Vector
from PythonRobotics.turtlebot import Robot
from PythonRobotics.graphics import World

show_animation = True


class Waypoints:
    """ navigation waypoints, collection of xy Vectors """

    def __init__(self, points: 'list[Vector]'):
        """ create waypoints. """
        self.points = [Vector(x, y) for x, y in points]
        self.next_idx = None

    def find_next_idx(self, xy: 'Vector') -> 'int':
        """ get  next waypoint """

        if self.next_idx is None:  # first time search
            # distance between points
            dst = [pt.distance(xy) for pt in self.points]
            idx = np.argmin(dst)
            self.next_idx = idx + 1
        else:
            # calculate distances to n and n+1 waypoints
            idx = self.next_idx
            dst_a = xy.distance(self.points[idx-1])
            dst_b = xy.distance(self.points[idx])
            if dst_a > dst_b:  # next point
                self.next_idx += 1

        return self.next_idx

    @property
    def target_reached(self):
        return not self.next_idx < len(self.points)


def point_on_line(a, b, p):
    """ project point p to a line defined by points a and b """
    ap = p - a
    ab = b - a
    result = a + ap.dot(ab) / ab.dot(ab) * ab
    return result


def main():

    world = World((-1, -1, 3, 3))

    # sim parameters
    t_sim = 10.0  # simulation time [sec]
    dt = 0.1  # timestep
    n_steps = int(t_sim / dt)

    # create waypoints
    coord = [(0, 0), (0, 2), (2, 2), (2, 0), (2, -5)]
    world.plot_path(coord)
    wp = Waypoints(coord)
    print('Waypoints: ', wp.points)

    # create robot
    bot = Robot(phi=math.pi/4)
    bot.plot_scale = 100
    bot.set_velocity(3., math.pi/2)

    # simulate
    for _ in range(n_steps):
        bot.step(dt)
        world.robot_step(bot.state)

        time.sleep(0.01)

    print('Simulation result:\n', bot.states_df())

    print('close simulation window to exit...')
    turtle.done()


if __name__ == "__main__":
    main()
