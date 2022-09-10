"""

eta^3 polynomials trajectory planner

author: Joe Dinius, Ph.D (https://jwdinius.github.io)
        Atsushi Sakai (@Atsushi_twi)

Refs:
- https://jwdinius.github.io/blog/2018/eta3traj
- [eta^3-Splines for the Smooth Path Generation of Wheeled Mobile Robots]
(https://ieeexplore.ieee.org/document/4339545/)

"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from Eta3SplinePath.eta3_spline_path import Eta3Path, Eta3PathSegment

show_animation = True


class MaxVelocityNotReached(Exception):
    def __init__(self, actual_vel, max_vel):
        self.message = 'Actual velocity {} does not equal desired max velocity {}!'.format(
            actual_vel, max_vel)


class eta3_trajectory(Eta3Path):
    """
    eta3_trajectory

    input
        segments: list of `eta3_trajectory_segment` instances defining a continuous trajectory
    """

    def __init__(self, segments, max_vel, v0=0.0, a0=0.0, max_accel=2.0, max_jerk=5.0):
        # ensure that all inputs obey the assumptions of the model
        assert max_vel > 0 and v0 >= 0 and a0 >= 0 and max_accel > 0 and max_jerk > 0 \
            and a0 <= max_accel and v0 <= max_vel
        super(eta3_trajectory, self).__init__(segments=segments)
        self.total_length = sum([s.segment_length for s in self.segments])
        self.max_vel = float(max_vel)
        self.v0 = float(v0)
        self.a0 = float(a0)
        self.max_accel = float(max_accel)
        self.max_jerk = float(max_jerk)
        length_array = np.array([s.segment_length for s in self.segments])
        # add a zero to the beginning for finding the correct segment_id
        self.cum_lengths = np.concatenate(
            (np.array([0]), np.cumsum(length_array)))
        # compute velocity profile on top of the path
        self.velocity_profile()
        self.ui_prev = 0
        self.prev_seg_id = 0

    def velocity_profile(self):
        r"""                  /~~~~~----------------\
                             /                       \
                            /                         \
                           /                           \
                          /                             \
        (v=v0, a=a0) ~~~~~                               \
                                                          \
                                                           \ ~~~~~ (vf=0, af=0)
                     pos.|pos.|neg.|   cruise at    |neg.| neg. |neg.
                     max |max.|max.|     max.       |max.| max. |max.
                     jerk|acc.|jerk|    velocity    |jerk| acc. |jerk
            index     0    1    2      3 (optional)   4     5     6
        """
        # delta_a: accel change from initial position to end of maximal jerk section
        delta_a = self.max_accel - self.a0
        # t_s1: time of traversal of maximal jerk section
        t_s1 = delta_a / self.max_jerk
        # v_s1: velocity at the end of the maximal jerk section
        v_s1 = self.v0 + self.a0 * t_s1 + self.max_jerk * t_s1**2 / 2.
        # s_s1: length of the maximal jerk section
        s_s1 = self.v0 * t_s1 + self.a0 * t_s1**2 / 2. + self.max_jerk * t_s1**3 / 6.
        # t_sf: time of traversal  of final section, which is also maximal jerk, but has final velocity 0
        t_sf = self.max_accel / self.max_jerk
        # v_sf: velocity at beginning of final section
        v_sf = self.max_jerk * t_sf**2 / 2.
        # s_sf: length of final section
        s_sf = self.max_jerk * t_sf**3 / 6.
        # solve for the maximum achievable velocity based on the kinematic limits imposed by max_accel and max_jerk
        # this leads to a quadratic equation in v_max: a*v_max**2 + b*v_max + c = 0
        a = 1 / self.max_accel
        b = 3. * self.max_accel / (2. * self.max_jerk) + v_s1 / self.max_accel - (
            self.max_accel**2 / self.max_jerk + v_s1) / self.max_accel
        c = s_s1 + s_sf - self.total_length - 7. * self.max_accel**3 / (3. * self.max_jerk**2) \
            - v_s1 * (self.max_accel / self.max_jerk + v_s1 / self.max_accel) \
            + (self.max_accel**2 / self.max_jerk + v_s1 /
               self.max_accel)**2 / (2. * self.max_accel)
        v_max = (-b + np.sqrt(b**2 - 4. * a * c)) / (2. * a)

        # v_max represents the maximum velocity that could be attained if there was no cruise period
        # (i.e. driving at constant speed without accelerating or jerking)
        # if this velocity is less than our desired max velocity, the max velocity needs to be updated
        if self.max_vel > v_max:
            # when this condition is tripped, there will be no cruise period (s_cruise=0)
            self.max_vel = v_max

        # setup arrays to store values at END of trajectory sections
        self.times = np.zeros((7,))
        self.vels = np.zeros((7,))
        self.seg_lengths = np.zeros((7,))

        # Section 0: max jerk up to max acceleration
        self.times[0] = t_s1
        self.vels[0] = v_s1
        self.seg_lengths[0] = s_s1

        # Section 1: accelerate at max_accel
        index = 1
        # compute change in velocity over the section
        delta_v = (self.max_vel - self.max_jerk * (self.max_accel /
                                                   self.max_jerk)**2 / 2.) - self.vels[index - 1]
        self.times[index] = delta_v / self.max_accel
        self.vels[index] = self.vels[index - 1] + \
            self.max_accel * self.times[index]
        self.seg_lengths[index] = self.vels[index - 1] * \
            self.times[index] + self.max_accel * self.times[index]**2 / 2.

        # Section 2: decrease acceleration (down to 0) until max speed is hit
        index = 2
        self.times[index] = self.max_accel / self.max_jerk
        self.vels[index] = self.vels[index - 1] + self.max_accel * self.times[index] \
            - self.max_jerk * self.times[index]**2 / 2.

        # as a check, the velocity at the end of the section should be self.max_vel
        if not np.isclose(self.vels[index], self.max_vel):
            raise MaxVelocityNotReached(self.vels[index], self.max_vel)

        self.seg_lengths[index] = self.vels[index - 1] * self.times[index] + self.max_accel * self.times[index]**2 / 2. \
            - self.max_jerk * self.times[index]**3 / 6.

        # Section 3: will be done last

        # Section 4: apply min jerk until min acceleration is hit
        index = 4
        self.times[index] = self.max_accel / self.max_jerk
        self.vels[index] = self.max_vel - \
            self.max_jerk * self.times[index]**2 / 2.
        self.seg_lengths[index] = self.max_vel * self.times[index] - \
            self.max_jerk * self.times[index]**3 / 6.

        # Section 5: continue deceleration at max rate
        index = 5
        # compute velocity change over sections
        delta_v = self.vels[index - 1] - v_sf
        self.times[index] = delta_v / self.max_accel
        self.vels[index] = self.vels[index - 1] - \
            self.max_accel * self.times[index]
        self.seg_lengths[index] = self.vels[index - 1] * \
            self.times[index] - self.max_accel * self.times[index]**2 / 2.

        # Section 6(final): max jerk to get to zero velocity and zero acceleration simultaneously
        index = 6
        self.times[index] = t_sf
        self.vels[index] = self.vels[index - 1] - self.max_jerk * t_sf**2 / 2.

        try:
            assert np.isclose(self.vels[index], 0)
        except AssertionError as e:
            print('The final velocity {} is not zero'.format(self.vels[index]))
            raise e

        self.seg_lengths[index] = s_sf

        if self.seg_lengths.sum() < self.total_length:
            index = 3
            # the length of the cruise section is whatever length hasn't already been accounted for
            # NOTE: the total array self.seg_lengths is summed because the entry for the cruise segment is
            # initialized to 0!
            self.seg_lengths[index] = self.total_length - \
                self.seg_lengths.sum()
            self.vels[index] = self.max_vel
            self.times[index] = self.seg_lengths[index] / self.max_vel

        # make sure that all of the times are positive, otherwise the kinematic limits
        # chosen cannot be enforced on the path
        assert(np.all(self.times >= 0))
        self.total_time = self.times.sum()

    def get_interp_param(self, seg_id, s, ui, tol=0.001):
        def f(u):
            return self.segments[seg_id].f_length(u)[0] - s

        def fprime(u):
            return self.segments[seg_id].s_dot(u)
        while (0 <= ui <= 1) and abs(f(ui)) > tol:
            ui -= f(ui) / fprime(ui)
        ui = max(0, min(ui, 1))
        return ui

    def calc_traj_point(self, time):
        # compute velocity at time
        if time <= self.times[0]:
            linear_velocity = self.v0 + self.max_jerk * time**2 / 2.
            s = self.v0 * time + self.max_jerk * time**3 / 6
            linear_accel = self.max_jerk * time
        elif time <= self.times[:2].sum():
            delta_t = time - self.times[0]
            linear_velocity = self.vels[0] + self.max_accel * delta_t
            s = self.seg_lengths[0] + self.vels[0] * \
                delta_t + self.max_accel * delta_t**2 / 2.
            linear_accel = self.max_accel
        elif time <= self.times[:3].sum():
            delta_t = time - self.times[:2].sum()
            linear_velocity = self.vels[1] + self.max_accel * \
                delta_t - self.max_jerk * delta_t**2 / 2.
            s = self.seg_lengths[:2].sum() + self.vels[1] * delta_t + self.max_accel * delta_t**2 / 2. \
                - self.max_jerk * delta_t**3 / 6.
            linear_accel = self.max_accel - self.max_jerk * delta_t
        elif time <= self.times[:4].sum():
            delta_t = time - self.times[:3].sum()
            linear_velocity = self.vels[3]
            s = self.seg_lengths[:3].sum() + self.vels[3] * delta_t
            linear_accel = 0.
        elif time <= self.times[:5].sum():
            delta_t = time - self.times[:4].sum()
            linear_velocity = self.vels[3] - self.max_jerk * delta_t**2 / 2.
            s = self.seg_lengths[:4].sum() + self.vels[3] * \
                delta_t - self.max_jerk * delta_t**3 / 6.
            linear_accel = -self.max_jerk * delta_t
        elif time <= self.times[:-1].sum():
            delta_t = time - self.times[:5].sum()
            linear_velocity = self.vels[4] - self.max_accel * delta_t
            s = self.seg_lengths[:5].sum() + self.vels[4] * \
                delta_t - self.max_accel * delta_t**2 / 2.
            linear_accel = -self.max_accel
        elif time < self.times.sum():
            delta_t = time - self.times[:-1].sum()
            linear_velocity = self.vels[5] - self.max_accel * \
                delta_t + self.max_jerk * delta_t**2 / 2.
            s = self.seg_lengths[:-1].sum() + self.vels[5] * delta_t - self.max_accel * delta_t**2 / 2. \
                + self.max_jerk * delta_t**3 / 6.
            linear_accel = -self.max_accel + self.max_jerk * delta_t
        else:
            linear_velocity = 0.
            s = self.total_length
            linear_accel = 0.
        seg_id = np.max(np.argwhere(self.cum_lengths <= s))

        # will happen at the end of the segment
        if seg_id == len(self.segments):
            seg_id -= 1
            ui = 1
        else:
            # compute interpolation parameter using length from current segment's starting point
            curr_segment_length = s - self.cum_lengths[seg_id]
            ui = self.get_interp_param(
                seg_id=seg_id, s=curr_segment_length, ui=self.ui_prev)

        if not seg_id == self.prev_seg_id:
            self.ui_prev = 0
        else:
            self.ui_prev = ui

        self.prev_seg_id = seg_id
        # compute angular velocity of current point= (ydd*xd - xdd*yd) / (xd**2 + yd**2)
        d = self.segments[seg_id].calc_deriv(ui, order=1)
        dd = self.segments[seg_id].calc_deriv(ui, order=2)
        # su - the rate of change of arclength wrt u
        su = self.segments[seg_id].s_dot(ui)
        if not np.isclose(su, 0.) and not np.isclose(linear_velocity, 0.):
            # ut - time-derivative of interpolation parameter u
            ut = linear_velocity / su
            # utt - time-derivative of ut
            utt = linear_accel / su - \
                (d[0] * dd[0] + d[1] * dd[1]) / su**2 * ut
            xt = d[0] * ut
            yt = d[1] * ut
            xtt = dd[0] * ut**2 + d[0] * utt
            ytt = dd[1] * ut**2 + d[1] * utt
            angular_velocity = (ytt * xt - xtt * yt) / linear_velocity**2
        else:
            angular_velocity = 0.

        # combine path point with orientation and velocities
        pos = self.segments[seg_id].calc_point(ui)
        state = np.array([pos[0], pos[1], np.arctan2(
            d[1], d[0]), linear_velocity, angular_velocity])
        return state


def test1(max_vel=0.5):

    for i in range(10):
        trajectory_segments = []
        # segment 1: lane-change curve
        start_pose = [0, 0, 0]
        end_pose = [4, 3.0, 0]
        # NOTE: The ordering on kappa is [kappa_A, kappad_A, kappa_B, kappad_B], with kappad_* being the curvature derivative
        kappa = [0, 0, 0, 0]
        eta = [i, i, 0, 0, 0, 0]
        trajectory_segments.append(Eta3PathSegment(
            start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

        traj = eta3_trajectory(trajectory_segments,
                               max_vel=max_vel, max_accel=0.5)

        # interpolate at several points along the path
        times = np.linspace(0, traj.total_time, 101)
        state = np.empty((5, times.size))
        for j, t in enumerate(times):
            state[:, j] = traj.calc_traj_point(t)

        if show_animation:  # pragma: no cover
            # plot the path
            plt.plot(state[0, :], state[1, :])
            plt.pause(1.0)

    plt.show()
    if show_animation:
        plt.close("all")


def test2(max_vel=0.5):

    for i in range(10):
        trajectory_segments = []
        # segment 1: lane-change curve
        start_pose = [0, 0, 0]
        end_pose = [4, 3.0, 0]
        # NOTE: The ordering on kappa is [kappa_A, kappad_A, kappa_B, kappad_B], with kappad_* being the curvature derivative
        kappa = [0, 0, 0, 0]
        # NOTE: INTEGRATOR ERROR EXPLODES WHEN eta[:1] IS ZERO!
        # was: eta = [0, 0, (i - 5) * 20, (5 - i) * 20, 0, 0], now is:
        eta = [0.1, 0.1, (i - 5) * 20, (5 - i) * 20, 0, 0]
        trajectory_segments.append(Eta3PathSegment(
            start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

        traj = eta3_trajectory(trajectory_segments,
                               max_vel=max_vel, max_accel=0.5)

        # interpolate at several points along the path
        times = np.linspace(0, traj.total_time, 101)
        state = np.empty((5, times.size))
        for j, t in enumerate(times):
            state[:, j] = traj.calc_traj_point(t)

        if show_animation:
            # plot the path
            plt.plot(state[0, :], state[1, :])
            plt.pause(1.0)

    plt.show()
    if show_animation:
        plt.close("all")


def test3(max_vel=2.0):
    trajectory_segments = []

    # segment 1: lane-change curve
    start_pose = [0, 0, 0]
    end_pose = [4, 1.5, 0]
    # NOTE: The ordering on kappa is [kappa_A, kappad_A, kappa_B, kappad_B], with kappad_* being the curvature derivative
    kappa = [0, 0, 0, 0]
    eta = [4.27, 4.27, 0, 0, 0, 0]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # segment 2: line segment
    start_pose = [4, 1.5, 0]
    end_pose = [5.5, 1.5, 0]
    kappa = [0, 0, 0, 0]
    # NOTE: INTEGRATOR ERROR EXPLODES WHEN eta[:1] IS ZERO!
    # was: eta = [0, 0, 0, 0, 0, 0], now is:
    eta = [0.5, 0.5, 0, 0, 0, 0]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # segment 3: cubic spiral
    start_pose = [5.5, 1.5, 0]
    end_pose = [7.4377, 1.8235, 0.6667]
    kappa = [0, 0, 1, 1]
    eta = [1.88, 1.88, 0, 0, 0, 0]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # segment 4: generic twirl arc
    start_pose = [7.4377, 1.8235, 0.6667]
    end_pose = [7.8, 4.3, 1.8]
    kappa = [1, 1, 0.5, 0]
    eta = [7, 10, 10, -10, 4, 4]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # segment 5: circular arc
    start_pose = [7.8, 4.3, 1.8]
    end_pose = [5.4581, 5.8064, 3.3416]
    kappa = [0.5, 0, 0.5, 0]
    eta = [2.98, 2.98, 0, 0, 0, 0]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # construct the whole path
    traj = eta3_trajectory(trajectory_segments,
                           max_vel=max_vel, max_accel=0.5, max_jerk=1)

    # interpolate at several points along the path
    times = np.linspace(0, traj.total_time, 1001)
    state = np.empty((5, times.size))
    for i, t in enumerate(times):
        state[:, i] = traj.calc_traj_point(t)

    # plot the path

    if show_animation:
        fig, ax = plt.subplots()
        x, y = state[0, :], state[1, :]
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segs = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segs, cmap=plt.get_cmap('inferno'))
        ax.set_xlim(np.min(x) - 1, np.max(x) + 1)
        ax.set_ylim(np.min(y) - 1, np.max(y) + 1)
        lc.set_array(state[3, :])
        lc.set_linewidth(3)
        ax.add_collection(lc)
        axcb = fig.colorbar(lc)
        axcb.set_label('velocity(m/s)')
        ax.set_title('Trajectory')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.pause(1.0)

        fig1, ax1 = plt.subplots()
        ax1.plot(times, state[3, :], 'b-')
        ax1.set_xlabel('time(s)')
        ax1.set_ylabel('velocity(m/s)', color='b')
        ax1.tick_params('y', colors='b')
        ax1.set_title('Control')
        ax2 = ax1.twinx()
        ax2.plot(times, state[4, :], 'r-')
        ax2.set_ylabel('angular velocity(rad/s)', color='r')
        ax2.tick_params('y', colors='r')
        fig.tight_layout()
        plt.show()


def main():
    """
    recreate path from reference (see Table 1)
    """
    # test1()
    # test2()
    test3()


if __name__ == '__main__':
    main()
