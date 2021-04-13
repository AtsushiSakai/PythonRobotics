#!/usr/bin/env python
"""

Simulator of a differantial drive robot, Turtlebot3 - like

Author  - Jev Kuznetsov

"""

from collections import namedtuple
from math import sin, cos, degrees
import turtle
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


show_animation = True
plot_result = False


# define state object
state_fields = ['x', 'y', 'phi', 'v', 'omega', 't']
State = namedtuple('State', state_fields, defaults=(0.,) * len(state_fields))


class Robot:
    """
    Robot kinematics simulator
    """
    # numbers for Robotis Turtlebot3
    v_max = 0.22  # [m/s]
    omega_max = 2.84  # [rad/s]
    accel_angular = 0.1  # [rad/s2]
    accel_linear = 1  # [m/s2]
    _wheel_diameter = 66e-3  # [m]
    _wheel_distance = 160e-3  # [m]

    # plot settings
    plot_scale = 1000

    def __init__(self,
                 x=0.,  # x-position
                 y=0.,
                 phi=0.,
                 animate=False):
        """ initiate robot at given location and orientation """
        self.states = [State(x, y, phi)]
        self._omega_target = 0.
        self._v_target = 0.

        if animate:
            self.screen = turtle.Screen()
            self.screen.setup(640, 480)
            self.turtle = turtle.Turtle()
        else:
            self.turtle = None

    def set_velocity(self, v, omega):
        """ set target velocities """
        self._omega_target = np.clip(omega, -self.omega_max, self.omega_max)
        self._v_target = np.clip(v, 0, self.v_max)

    def step(self, dt=0.1):
        """ perform simulation step with timestep dt """

        s = self.state  # start state

        # update linear speed
        delta_v = self.accel_linear * dt  # dv is an absolute number
        v_error = self._v_target - s.v
        if abs(v_error) > delta_v:
            v_new = s.v + np.sign(v_error) * delta_v
        else:
            v_new = s.v + v_error

        # update angular speed
        delta_omega = self.accel_angular * dt
        omega_error = self._omega_target - s.omega
        if abs(omega_error) > delta_omega:
            omega_new = s.omega + np.sign(omega_error) * delta_omega
        else:
            omega_new = s.omega + omega_error

        # create new state
        s_new = State(x=s.x + s.v*cos(s.phi)*dt,
                      y=s.y + s.v*sin(s.phi)*dt,
                      phi=s.phi + s.omega*dt,
                      t=s.t + dt,
                      v=v_new,
                      omega=omega_new)

        self.states.append(s_new)

        # animate if required
        if self.turtle is not None:
            self.turtle.setpos(s_new.x*self.plot_scale, s_new.y * self.plot_scale)
            self.turtle.setheading(degrees(s_new.phi))

    def states_df(self):
        """ states as DataFrame """
        cols = state_fields[:-1]
        data = {}

        for var_name in cols:
            data[var_name] = [getattr(s, var_name) for s in self.states]

        t = pd.Index(name='time', data=[s.t for s in self.states])
        df = pd.DataFrame(data, index=t)

        return df

    @property
    def state(self):
        """ last known state """
        return self.states[-1]

    def __repr__(self):
        return f'Turtlebot {self.state})'


def main():
    """ demonstrate functionality """

    sim = Robot(animate=show_animation)

    # high acceleration ~ instantaneous velocity
    sim.accel_angular = 1000  # high angular acceleration
    sim.accel_linear = 1  # constant linear acceleration
    sim.v_max = 10  # max linear velocity
    sim.omega_max = 2*np.pi  # max angular velocity

    omega = 2*np.pi  # rad/s

    # calculate path
    t_sim = 1.0  # simulation time [sec]
    dt = 0.01  # timestep
    n_steps = int(t_sim / dt)

    sim.set_velocity(1.0, omega)

    for _ in range(n_steps):
        sim.step(dt)

    states = sim.states_df()
    print('Simulation result:\n', states)

    # plot data
    if plot_result:
        plt.style.use('seaborn-whitegrid')

        plt.cla()
        plt.tight_layout()

        plt.subplot(2, 2, 1)
        states.v.plot()
        plt.title('v (linear velocity)')

        plt.subplot(2, 2, 2)
        plt.plot(states.x, states.y, 'x-',)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('position')

        plt.subplot(2, 2, 3)
        states.omega.plot()
        plt.title('omega (angular velocity)')
        plt.ylabel('[rad/s]')

        plt.subplot(2, 2, 4)
        states.phi.plot()
        plt.title('phi')
        plt.ylabel('phi [rad]')

        plt.subplots_adjust(hspace=0.4)

        plt.show()

    if show_animation:
        print('close plot window to continue....')
        turtle.done()


if __name__ == '__main__':
    print('Turtlebot simulation demo starting...')
    main()
