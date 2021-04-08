#!/usr/bin/env python
"""

Simulator of a differantial drive robot, Turtlebot3 - like

Author  - Jev Kuznetsov

"""

import numpy as np

from numpy import sin, cos, sign
import pandas as pd

from collections import namedtuple

show_animation = True

# define state object
state_fields = ['x', 'y', 'phi', 'v', 'omega', 't']
State = namedtuple('State', state_fields, defaults=(0.,) * len(state_fields))


class Robot:
    """
    Robot kinematics simulator
    """
    # numbers for Robotis Turtlebot3
    _v_max = 0.22  # [m/s]
    _omega_max = 2.84  # [rad/s]
    _accel_angular = 0.1  # [rad/s2]
    _accel_linear = 0.1  # [m/s2]
    _wheel_diameter = 66e-3  # [m]
    _wheel_distance = 160e-3  # [m]

    def __init__(self,
                 x=0.,  # x-position
                 y=0.,
                 phi=0.,
                 name='Turtlebot3'):
        """ initiate robot at given location and orientation """
        self.name = name
        self._states = [State(x, y, phi)]

        self._omega_target = 0.
        self._v_target = 0.

    def set_velocity(self, v, omega):
        """ set target velocities """
        self._omega_target = np.clip(omega, -self._omega_max, self._omega_max)
        self._v_target = np.clip(v, 0, self._v_max)

    def step(self, dt=0.1):
        """ perform simulation step with timestep dt """

        s = self.state  # start state

        # update linear speed
        delta_v = self._accel_linear * dt  # dv is an absolute number
        v_error = self._v_target - s.v
        if abs(v_error) > delta_v:
            v_new = s.v + sign(v_error) * delta_v
        else:
            v_new = s.v + v_error

        # update angular speed
        delta_omega = self._accel_angular * dt
        omega_error = self._omega_target - s.omega
        if abs(omega_error) > delta_omega:
            omega_new = s.omega + sign(omega_error) * delta_omega
        else:
            omega_new = s.omega + omega_error

        # create new state
        s_new = State(x=s.x + s.v*cos(s.phi)*dt,
                      y=s.y + s.v*sin(s.phi)*dt,
                      phi=s.phi + s.omega*dt,
                      t=s.t + dt,
                      v=v_new,
                      omega=omega_new)

        self._states.append(s_new)

    @property
    def states(self):
        """ get history of all states as pandas DataFrame """

        cols = state_fields[:-1]
        data = {}

        for var_name in cols:
            data[var_name] = [getattr(s, var_name) for s in self._states]

        t = pd.Index(name='time', data=[s.t for s in self._states])

        df = pd.DataFrame(data, index=t)

        return df

    @property
    def state(self):
        """ last known state """
        return self._states[-1]

    def __repr__(self):
        return f'{self.name} {self.state})'


def main():
    """ demonstrate functionality """
    import matplotlib.pyplot as plt

    bot = Robot()

    # high acceleration ~ instantaneous velocity
    bot._accel_angular = 1000  # high angular acceleration
    bot._accel_linear = 1  # constant linear acceleration
    bot._v_max = 10  # max linear velocity
    bot._omega_max = 2*np.pi  # max angular velocity

    omega = 2*np.pi  # rad/s

    # calculate path
    t_sim = 1.0  # simulation time [sec]
    dt = 0.01  # timestep
    n_steps = int(t_sim / dt)

    bot.set_velocity(1.0, omega)

    for _ in range(n_steps):
        bot.step(dt)

    print('Simulation result:\n', bot.states)

    # plot data
    if show_animation:
        plt.style.use('seaborn-whitegrid')

        plt.cla()
        plt.tight_layout()

        plt.subplot(2, 2, 1)
        bot.states.v.plot()
        plt.title('v (linear velocity)')

        plt.subplot(2, 2, 2)
        plt.plot(bot.states.x, bot.states.y, 'x-',)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('position')

        plt.subplot(2, 2, 3)
        bot.states.omega.plot()
        plt.title('omega (angular velocity)')
        plt.ylabel('[rad/s]')

        plt.subplot(2, 2, 4)
        bot.states.phi.plot()
        plt.title('phi')
        plt.ylabel('phi [rad]')

        plt.subplots_adjust(hspace=0.4)

        plt.show()


if __name__ == '__main__':
    print('Turtlebot simulation demo starting...')
    main()
