#!/usr/bin/env python
"""
 Pure pursuit controller V2

 Copyright (c) 2021 AIGRO B.V. - Jev Kuznetsov
"""

from collections import UserList
import time
import math
import turtle
import numpy as np
from PythonRobotics.vectors import Vector
from PythonRobotics.turtlebot import Robot
from PythonRobotics.graphics import World

show_animation = True


class Waypoints(UserList):
    """ navigation waypoints, collection of xy Vectors """

    def __init__(self, *args, **kwargs):
        """ create waypoints. """
        super().__init__(*args, **kwargs)

        self.next_idx = None

    def find_next_idx(self, xy: 'Vector') -> 'int':
        """ get  next waypoint """

        if self.next_idx is None:  # first time search
            # distance between points
            dst = [pt.distance(xy) for pt in self.data]
            idx = np.argmin(dst)
            self.next_idx = idx + 1
        else:
            # calculate distances to n and n+1 waypoints
            idx = self.next_idx
            dst_a = xy.distance(self.data[idx-1])
            dst_b = xy.distance(self.data[idx])
            if dst_a > dst_b:  # next point
                self.next_idx += 1

        return self.next_idx

    @property
    def target_reached(self):
        return not self.next_idx < len(self.data)


def point_on_line(a, b, p):
    """ project point p to a line defined by points a and b """
    ap = p - a
    ab = b - a
    result = a + ap.dot(ab) / ab.dot(ab) * ab
    return result


def target_position(state, wp_a, wp_b,  look_ahead=1) -> 'Vector':
    """ calculate robot target position """

    # future position
    d = Vector(state.x, state.y) + Vector.from_polar(look_ahead, state.phi)

    # project to line
    e = point_on_line(wp_a, wp_b, d)

    return e


def main():

    world = World((-1, -1, 3, 3))
    world.add_marker('target', trace=True)

    # sim parameters
    t_sim = 10.0  # simulation time [sec]
    dt = 0.1  # timestep
    n_steps = int(t_sim / dt)
    # create waypoints
    coord = [Vector(xy) for xy in [(0, 0), (0, 2), (2, 2), (2, 0), (2, -5)]]
    world.plot_path(coord)
    waypoints = Waypoints(coord)
    print('Waypoints: ', waypoints)

    # create robot
    bot = Robot(phi=math.pi/4)
    bot.set_velocity(3., math.pi/2)

    # simulate
    for i_step in range(n_steps):
        bot.step(dt)

        idx = waypoints.find_next_idx(bot.xy)
        target = target_position(bot.state, waypoints[idx-1], waypoints[idx])

        world.move_robot(bot.state)

        world.move_marker('target', target)
        print(f"step[{i_step}] next_wp:{waypoints.next_idx}")
        time.sleep(0.1)

    print('Simulation result:\n', bot.states_df())

    print('close simulation window to exit...')
    turtle.done()


if __name__ == "__main__":
    main()
