import numpy as np

import conftest  # Add root path to sys.path
from PathPlanning.ReedsSheppPath import reeds_shepp_path_planning as m


def check_edge_condition(px, py, pyaw, start_x, start_y, start_yaw, end_x,
                         end_y, end_yaw):
    assert (abs(px[0] - start_x) <= 0.01)
    assert (abs(py[0] - start_y) <= 0.01)
    assert (abs(pyaw[0] - start_yaw) <= 0.01)
    assert (abs(px[-1] - end_x) <= 0.01)
    assert (abs(py[-1] - end_y) <= 0.01)
    assert (abs(pyaw[-1] - end_yaw) <= 0.01)


def check_path_length(px, py, lengths):
    sum_len = sum(abs(length) for length in lengths)
    dpx = np.diff(px)
    dpy = np.diff(py)
    actual_len = sum(
        np.hypot(dx, dy) for (dx, dy) in zip(dpx, dpy))
    diff_len = sum_len - actual_len
    assert (diff_len <= 0.01)


def test1():
    m.show_animation = False
    m.main()


def test2():
    N_TEST = 10
    np.random.seed(1234)

    for i in range(N_TEST):
        start_x = (np.random.rand() - 0.5) * 10.0  # [m]
        start_y = (np.random.rand() - 0.5) * 10.0  # [m]
        start_yaw = np.deg2rad((np.random.rand() - 0.5) * 180.0)  # [rad]

        end_x = (np.random.rand() - 0.5) * 10.0  # [m]
        end_y = (np.random.rand() - 0.5) * 10.0  # [m]
        end_yaw = np.deg2rad((np.random.rand() - 0.5) * 180.0)  # [rad]

        curvature = 1.0 / (np.random.rand() * 5.0)

        px, py, pyaw, mode, lengths = m.reeds_shepp_path_planning(
            start_x, start_y, start_yaw,
            end_x, end_y, end_yaw, curvature)

        check_edge_condition(px, py, pyaw, start_x, start_y, start_yaw, end_x,
                             end_y, end_yaw)
        check_path_length(px, py, lengths)


def test_identical_start_and_goal_pose():
    for pose in [(0.0, 0.0, 0.0), (1.2, -3.4, 0.7)]:
        px, py, pyaw, _, lengths = m.reeds_shepp_path_planning(
            *pose, *pose, maxc=1.0
        )

        assert sum(np.abs(lengths)) == 0.0
        assert np.all(np.isfinite(px))
        assert np.all(np.isfinite(py))
        assert np.all(np.isfinite(pyaw))
        assert np.all(np.isfinite(lengths))
        np.testing.assert_allclose(px, pose[0], atol=1e-10)
        np.testing.assert_allclose(py, pose[1], atol=1e-10)
        np.testing.assert_allclose(pyaw, pose[2], atol=1e-10)


def test_near_singular_path_formula():
    for distance in [1e-6, 1e-12]:
        is_valid, lengths, modes = m.left_right_x_left(
            distance, 0.0, 0.0
        )

        assert is_valid
        assert np.all(np.isfinite(lengths))
        np.testing.assert_allclose(lengths[1], distance / 2.0, rtol=1e-12)
        assert modes == ["L", "R", "L"]


if __name__ == '__main__':
    conftest.run_this_test(__file__)
