"""

\eta^3 polynomials planner

author: Joe Dinius, Ph.D (https://jwdinius.github.io)
        Atsushi Sakai (@Atsushi_twi)

Ref:

- [\eta^3-Splines for the Smooth Path Generation of Wheeled Mobile Robots](https://ieeexplore.ieee.org/document/4339545/)

"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad

# NOTE: *_pose is a 3-array: 0 - x coord, 1 - y coord, 2 - orientation angle \theta

show_animation = True


class eta3_path(object):
    """
    eta3_path

    input
        segments: list of `eta3_path_segment` instances definining a continuous path
    """

    def __init__(self, segments):
        # ensure input has the correct form
        assert(isinstance(segments, list) and isinstance(
            segments[0], eta3_path_segment))
        # ensure that each segment begins from the previous segment's end (continuity)
        for r, s in zip(segments[:-1], segments[1:]):
            assert(np.array_equal(r.end_pose, s.start_pose))
        self.segments = segments
    """
    eta3_path::calc_path_point

    input
        normalized interpolation point along path object, 0 <= u <= len(self.segments)
    returns
        2d (x,y) position vector
    """

    def calc_path_point(self, u):
        assert(u >= 0 and u <= len(self.segments))
        if np.isclose(u, len(self.segments)):
            segment_idx = len(self.segments) - 1
            u = 1.
        else:
            segment_idx = int(np.floor(u))
            u -= segment_idx
        return self.segments[segment_idx].calc_point(u)


class eta3_path_segment(object):
    """
    eta3_path_segment - constructs an eta^3 path segment based on desired shaping, eta, and curvature vector, kappa.
                        If either, or both, of eta and kappa are not set during initialization, they will
                        default to zeros.

    input
        start_pose - starting pose array  (x, y, \theta)
        end_pose - ending pose array (x, y, \theta)
        eta - shaping parameters, default=None
        kappa - curvature parameters, default=None
    """

    def __init__(self, start_pose, end_pose, eta=None, kappa=None):
        # make sure inputs are of the correct size
        assert(len(start_pose) == 3 and len(start_pose) == len(end_pose))
        self.start_pose = start_pose
        self.end_pose = end_pose
        # if no eta is passed, initialize it to array of zeros
        if not eta:
            eta = np.zeros((6,))
        else:
            # make sure that eta has correct size
            assert(len(eta) == 6)
        # if no kappa is passed, initialize to array of zeros
        if not kappa:
            kappa = np.zeros((4,))
        else:
            assert(len(kappa) == 4)
        # set up angle cosines and sines for simpler computations below
        ca = np.cos(start_pose[2])
        sa = np.sin(start_pose[2])
        cb = np.cos(end_pose[2])
        sb = np.sin(end_pose[2])
        # 2 dimensions (x,y) x 8 coefficients per dimension
        self.coeffs = np.empty((2, 8))
        # constant terms (u^0)
        self.coeffs[0, 0] = start_pose[0]
        self.coeffs[1, 0] = start_pose[1]
        # linear (u^1)
        self.coeffs[0, 1] = eta[0] * ca
        self.coeffs[1, 1] = eta[0] * sa
        # quadratic (u^2)
        self.coeffs[0, 2] = 1. / 2 * eta[2] * \
            ca - 1. / 2 * eta[0]**2 * kappa[0] * sa
        self.coeffs[1, 2] = 1. / 2 * eta[2] * \
            sa + 1. / 2 * eta[0]**2 * kappa[0] * ca
        # cubic (u^3)
        self.coeffs[0, 3] = 1. / 6 * eta[4] * ca - 1. / 6 * \
            (eta[0]**3 * kappa[1] + 3. * eta[0] * eta[2] * kappa[0]) * sa
        self.coeffs[1, 3] = 1. / 6 * eta[4] * sa + 1. / 6 * \
            (eta[0]**3 * kappa[1] + 3. * eta[0] * eta[2] * kappa[0]) * ca
        # quartic (u^4)
        self.coeffs[0, 4] = 35. * (end_pose[0] - start_pose[0]) - (20. * eta[0] + 5 * eta[2] + 2. / 3 * eta[4]) * ca \
            + (5. * eta[0]**2 * kappa[0] + 2. / 3 * eta[0]**3 * kappa[1] + 2. * eta[0] * eta[2] * kappa[0]) * sa \
            - (15. * eta[1] - 5. / 2 * eta[3] + 1. / 6 * eta[5]) * cb \
            - (5. / 2 * eta[1]**2 * kappa[2] - 1. / 6 * eta[1] **
               3 * kappa[3] - 1. / 2 * eta[1] * eta[3] * kappa[2]) * sb
        self.coeffs[1, 4] = 35. * (end_pose[1] - start_pose[1]) - (20. * eta[0] + 5. * eta[2] + 2. / 3 * eta[4]) * sa \
            - (5. * eta[0]**2 * kappa[0] + 2. / 3 * eta[0]**3 * kappa[1] + 2. * eta[0] * eta[2] * kappa[0]) * ca \
            - (15. * eta[1] - 5. / 2 * eta[3] + 1. / 6 * eta[5]) * sb \
            + (5. / 2 * eta[1]**2 * kappa[2] - 1. / 6 * eta[1] **
               3 * kappa[3] - 1. / 2 * eta[1] * eta[3] * kappa[2]) * cb
        # quintic (u^5)
        self.coeffs[0, 5] = -84. * (end_pose[0] - start_pose[0]) + (45. * eta[0] + 10. * eta[2] + eta[4]) * ca \
            - (10. * eta[0]**2 * kappa[0] + eta[0]**3 * kappa[1] + 3. * eta[0] * eta[2] * kappa[0]) * sa \
            + (39. * eta[1] - 7. * eta[3] + 1. / 2 * eta[5]) * cb \
            + (7. * eta[1]**2 * kappa[2] - 1. / 2 * eta[1]**3 *
               kappa[3] - 3. / 2 * eta[1] * eta[3] * kappa[2]) * sb
        self.coeffs[1, 5] = -84. * (end_pose[1] - start_pose[1]) + (45. * eta[0] + 10. * eta[2] + eta[4]) * sa \
            + (10. * eta[0]**2 * kappa[0] + eta[0]**3 * kappa[1] + 3. * eta[0] * eta[2] * kappa[0]) * ca \
            + (39. * eta[1] - 7. * eta[3] + 1. / 2 * eta[5]) * sb \
            - (7. * eta[1]**2 * kappa[2] - 1. / 2 * eta[1]**3 *
               kappa[3] - 3. / 2 * eta[1] * eta[3] * kappa[2]) * cb
        # sextic (u^6)
        self.coeffs[0, 6] = 70. * (end_pose[0] - start_pose[0]) - (36. * eta[0] + 15. / 2 * eta[2] + 2. / 3 * eta[4]) * ca \
            + (15. / 2 * eta[0]**2 * kappa[0] + 2. / 3 * eta[0]**3 * kappa[1] + 2. * eta[0] * eta[2] * kappa[0]) * sa \
            - (34. * eta[1] - 13. / 2 * eta[3] + 1. / 2 * eta[5]) * cb \
            - (13. / 2 * eta[1]**2 * kappa[2] - 1. / 2 * eta[1] **
               3 * kappa[3] - 3. / 2 * eta[1] * eta[3] * kappa[2]) * sb
        self.coeffs[1, 6] = 70. * (end_pose[1] - start_pose[1]) - (36. * eta[0] + 15. / 2 * eta[2] + 2. / 3 * eta[4]) * sa \
            - (15. / 2 * eta[0]**2 * kappa[0] + 2. / 3 * eta[0]**3 * kappa[1] + 2. * eta[0] * eta[2] * kappa[0]) * ca \
            - (34. * eta[1] - 13. / 2 * eta[3] + 1. / 2 * eta[5]) * sb \
            + (13. / 2 * eta[1]**2 * kappa[2] - 1. / 2 * eta[1] **
               3 * kappa[3] - 3. / 2 * eta[1] * eta[3] * kappa[2]) * cb
        # septic (u^7)
        self.coeffs[0, 7] = -20. * (end_pose[0] - start_pose[0]) + (10. * eta[0] + 2. * eta[2] + 1. / 6 * eta[4]) * ca \
            - (2. * eta[0]**2 * kappa[0] + 1. / 6 * eta[0]**3 * kappa[1] + 1. / 2 * eta[0] * eta[2] * kappa[0]) * sa \
            + (10. * eta[1] - 2. * eta[3] + 1. / 6 * eta[5]) * cb \
            + (2. * eta[1]**2 * kappa[2] - 1. / 6 * eta[1]**3 *
               kappa[3] - 1. / 2 * eta[1] * eta[3] * kappa[2]) * sb
        self.coeffs[1, 7] = -20. * (end_pose[1] - start_pose[1]) + (10. * eta[0] + 2. * eta[2] + 1. / 6 * eta[4]) * sa \
            + (2. * eta[0]**2 * kappa[0] + 1. / 6 * eta[0]**3 * kappa[1] + 1. / 2 * eta[0] * eta[2] * kappa[0]) * ca \
            + (10. * eta[1] - 2. * eta[3] + 1. / 6 * eta[5]) * sb \
            - (2. * eta[1]**2 * kappa[2] - 1. / 6 * eta[1]**3 *
               kappa[3] - 1. / 2 * eta[1] * eta[3] * kappa[2]) * cb
        
        s_dot = lambda u : np.linalg.norm(self.coeffs[:, 1:].dot(np.array([1, 2.*u, 3.*u**2, 4.*u**3, 5.*u**4, 6.*u**5, 7.*u**6])))
        self.segment_length = quad(lambda u: s_dot(u), 0, 1)[0]

    """
    eta3_path_segment::calc_point

    input
        u - parametric representation of a point along the segment, 0 <= u <= 1
    returns
        (x,y) of point along the segment
    """
    def calc_point(self, u):
        assert(u >= 0 and u <= 1)
        return self.coeffs.dot(np.array([1, u, u**2, u**3, u**4, u**5, u**6, u**7]))


def test1():

    for i in range(10):
        path_segments = []
        # segment 1: lane-change curve
        start_pose = [0, 0, 0]
        end_pose = [4, 3.0, 0]
        # NOTE: The ordering on kappa is [kappa_A, kappad_A, kappa_B, kappad_B], with kappad_* being the curvature derivative
        kappa = [0, 0, 0, 0]
        eta = [i, i, 0, 0, 0, 0]
        path_segments.append(eta3_path_segment(
            start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

        path = eta3_path(path_segments)

        # interpolate at several points along the path
        ui = np.linspace(0, len(path_segments), 1001)
        pos = np.empty((2, ui.size))
        for i, u in enumerate(ui):
            pos[:, i] = path.calc_path_point(u)

        if show_animation:
            # plot the path
            plt.plot(pos[0, :], pos[1, :])
            plt.pause(1.0)

    if show_animation:
        plt.close("all")


def test2():

    for i in range(10):
        path_segments = []
        # segment 1: lane-change curve
        start_pose = [0, 0, 0]
        end_pose = [4, 3.0, 0]
        # NOTE: The ordering on kappa is [kappa_A, kappad_A, kappa_B, kappad_B], with kappad_* being the curvature derivative
        kappa = [0, 0, 0, 0]
        eta = [0, 0, (i - 5) * 20, (5 - i) * 20, 0, 0]
        path_segments.append(eta3_path_segment(
            start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

        path = eta3_path(path_segments)

        # interpolate at several points along the path
        ui = np.linspace(0, len(path_segments), 1001)
        pos = np.empty((2, ui.size))
        for i, u in enumerate(ui):
            pos[:, i] = path.calc_path_point(u)

        if show_animation:
            # plot the path
            plt.plot(pos[0, :], pos[1, :])
            plt.pause(1.0)

    if show_animation:
        plt.close("all")


def test3():
    path_segments = []

    # segment 1: lane-change curve
    start_pose = [0, 0, 0]
    end_pose = [4, 1.5, 0]
    # NOTE: The ordering on kappa is [kappa_A, kappad_A, kappa_B, kappad_B], with kappad_* being the curvature derivative
    kappa = [0, 0, 0, 0]
    eta = [4.27, 4.27, 0, 0, 0, 0]
    path_segments.append(eta3_path_segment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # segment 2: line segment
    start_pose = [4, 1.5, 0]
    end_pose = [5.5, 1.5, 0]
    kappa = [0, 0, 0, 0]
    eta = [0, 0, 0, 0, 0, 0]
    path_segments.append(eta3_path_segment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # segment 3: cubic spiral
    start_pose = [5.5, 1.5, 0]
    end_pose = [7.4377, 1.8235, 0.6667]
    kappa = [0, 0, 1, 1]
    eta = [1.88, 1.88, 0, 0, 0, 0]
    path_segments.append(eta3_path_segment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # segment 4: generic twirl arc
    start_pose = [7.4377, 1.8235, 0.6667]
    end_pose = [7.8, 4.3, 1.8]
    kappa = [1, 1, 0.5, 0]
    eta = [7, 10, 10, -10, 4, 4]
    path_segments.append(eta3_path_segment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # segment 5: circular arc
    start_pose = [7.8, 4.3, 1.8]
    end_pose = [5.4581, 5.8064, 3.3416]
    kappa = [0.5, 0, 0.5, 0]
    eta = [2.98, 2.98, 0, 0, 0, 0]
    path_segments.append(eta3_path_segment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # construct the whole path
    path = eta3_path(path_segments)

    # interpolate at several points along the path
    ui = np.linspace(0, len(path_segments), 1001)
    pos = np.empty((2, ui.size))
    for i, u in enumerate(ui):
        pos[:, i] = path.calc_path_point(u)

    # plot the path

    if show_animation:
        plt.figure('Path from Reference')
        plt.plot(pos[0, :], pos[1, :])
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('Path')
        plt.pause(1.0)

        plt.show()


def main():
    """
    recreate path from reference (see Table 1)
    """
    test1()
    test2()
    test3()


if __name__ == '__main__':
    main()
