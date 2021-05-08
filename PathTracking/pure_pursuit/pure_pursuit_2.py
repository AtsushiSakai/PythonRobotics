#!/usr/bin/env python
"""
 Pure pursuit controller V2

 Copyright (c) 2021 AIGRO B.V. - Jev Kuznetsov
"""

from collections import UserList
import time
import math
import turtle
from typing import Tuple
import numpy as np
from PythonRobotics.vectors import Vector
from PythonRobotics.turtlebot import Robot
from PythonRobotics.graphics import World

show_animation = True

# sim parameters
WAYPOINTS_TYPE = 'square'
TARGET_SPEED = .1  # [m/s]
LOOK_AHEAD_1 = 0.1  # [m]  lenth of velocity vector
LOOK_AHEAD_2 = 0.2  # [m]  step forward along the path segment
Kp = 1.0  # controller gain
t_sim = 50.0  # simulation time [sec]
dt = 0.1  # timestep
SIM_DELAY = 0.01


class Waypoints(UserList):
    """ navigation waypoints, collection of xy Vectors """

    def __init__(self, *args, **kwargs):
        """ create waypoints. """
        super().__init__(*args, **kwargs)
        self.data = [Vector(d) for d in self.data]

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


def project_on_line(a: Vector, b: Vector, p: Vector) -> Vector:
    """ project point p to a line defined by points a and b """
    ap = p - a
    ab = b - a
    result = a + ap.dot(ab) / ab.dot(ab) * ab
    return result


def target_position(xy: Vector,
                    phi: float,
                    a: Vector, b: Vector,
                    velocity_vector=LOOK_AHEAD_1,
                    look_ahead=LOOK_AHEAD_2
                    ) -> Tuple[Vector, Vector]:
    """calculate look-ahead and target positions

    Args:
        xy (Vector): robot position
        phi (float): robot pose [rad]
        a (Vector): first waypoint
        b (Vector): second waypoint
        velocity_vector (float, optional): magnitude of velocity vector .
            Defaults to 1.0.
        look_ahead (float, optional): forward distance from projecton
            point to path.  Defaults to 1.0.

    Returns:
        Tuple[Vector, Vector, float]: target & look-ahead positions,
            and angle error
    """
    # point names consistent with https://www.geogebra.org/calculator/vh5d7jvy
    # future position
    v = Vector.from_polar(velocity_vector, phi)
    d = xy + v

    # project to line
    e = project_on_line(a, b, d)
    f = look_ahead*(b - a)/abs(b - a) + e

    # calculate angele error
    alpha = v.angle(f-xy)

    return f, d, alpha


def proportional_control(target: float, current: float) -> float:
    a = Kp * (target - current)

    return a


def generate_waypoints(key: str) -> Waypoints:
    """generate different types of waypoints

    Args:
        key (str): name of waypoint generator

    Returns:
        Waypoints: waypoints to follow
    """
    if key == 'simple':
        coord = [Vector(xy)
                 for xy in [(0, 0), (0, 2), (2, 2), (2, 0), (2, -3)]]
        waypoints = Waypoints(coord)
    elif key == 'sine':
        cx = np.arange(0, 5, 0.02)
        cy = [np.sin(ix / 0.1) * ix / 2.0 for ix in cx]
        waypoints = Waypoints(zip(cx, cy))
    elif key == 'square':
        w_tops = np.linspace(0.1, 1, 5)  # turn widhts
        waypoints = Waypoints([Vector(0, 0)])

        v_up = Vector(0, 1)
        waypoints.append(waypoints[-1] + 0.5*v_up)
        for w in reversed(w_tops):
            waypoints.append(waypoints[-1] + Vector(w, 0))
            waypoints.append(waypoints[-1] - v_up)
            waypoints.append(waypoints[-1] + Vector(0.4, 0))
            waypoints.append(waypoints[-1] + v_up)

    else:
        raise KeyError(f"Don't know how to generate waypoints for {key}")

    return waypoints


def main():

    # create visualisation
    world = World((0, -3, 5, 3))
    world.screen.title('Pure pursuit 2')
    world.add_marker('target')
    world.add_marker('future_xy', color="magenta")

    # create waypoints
    waypoints = generate_waypoints(WAYPOINTS_TYPE)
    print('Waypoints: ', waypoints)
    world.plot_path(waypoints)

    # create robot
    bot = Robot(phi=math.pi/4)

    i_step = 0
    # simulate
    while True:

        # advance simulation
        bot.step(dt)

        # calculate target point
        idx = waypoints.find_next_idx(bot.xy)
        if waypoints.target_reached:
            print('Target reached')
            break

        target, future_xy, alpha = target_position(
            bot.xy, bot.state.phi, waypoints[idx-1], waypoints[idx])

        # control robot
        omega = proportional_control(0, alpha)
        bot.set_velocity(TARGET_SPEED, omega)

        # visualise
        world.move_robot(bot.xy, bot.phi)
        world.move_marker('future_xy', future_xy)
        world.move_marker('target', target)
        print(
            f"step[{i_step}] next_wp:{waypoints.next_idx} \
            target:{target} omega:{omega:.2f}")
        time.sleep(SIM_DELAY)

        i_step += 1

        # check for exit
        if world.click_xy:
            print('Aborted by click')
            break

    print('Simulation result:\n', bot.states_df())

    print('close simulation window to exit...')
    turtle.done()


if __name__ == "__main__":
    main()
