import numpy as np

import conftest
from PathPlanning.DubinsPath import dubins_path_planning

np.random.seed(12345)


def check_edge_condition(px, py, pyaw, start_x, start_y, start_yaw, end_x,
                         end_y, end_yaw):
    assert (abs(px[0] - start_x) <= 0.01)
    assert (abs(py[0] - start_y) <= 0.01)
    assert (abs(pyaw[0] - start_yaw) <= 0.01)
    assert (abs(px[-1] - end_x) <= 0.01)
    assert (abs(py[-1] - end_y) <= 0.01)
    assert (abs(pyaw[-1] - end_yaw) <= 0.01)


def check_path_length(px, py, lengths):
    path_len = sum(
        [np.hypot(dx, dy) for (dx, dy) in zip(np.diff(px), np.diff(py))])
    assert (abs(path_len - sum(lengths)) <= 0.1)


def test_1():
    start_x = 1.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = np.deg2rad(45.0)  # [rad]

    end_x = -3.0  # [m]
    end_y = -3.0  # [m]
    end_yaw = np.deg2rad(-45.0)  # [rad]

    curvature = 1.0

    px, py, pyaw, mode, lengths = dubins_path_planning.dubins_path_planning(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)

    check_edge_condition(px, py, pyaw, start_x, start_y, start_yaw, end_x,
                         end_y, end_yaw)
    check_path_length(px, py, lengths)


def test_2():
    dubins_path_planning.show_animation = False
    dubins_path_planning.main()


def test_3():
    N_TEST = 10

    for i in range(N_TEST):
        start_x = (np.random.rand() - 0.5) * 10.0  # [m]
        start_y = (np.random.rand() - 0.5) * 10.0  # [m]
        start_yaw = np.deg2rad((np.random.rand() - 0.5) * 180.0)  # [rad]

        end_x = (np.random.rand() - 0.5) * 10.0  # [m]
        end_y = (np.random.rand() - 0.5) * 10.0  # [m]
        end_yaw = np.deg2rad((np.random.rand() - 0.5) * 180.0)  # [rad]

        curvature = 1.0 / (np.random.rand() * 5.0)

        px, py, pyaw, mode, lengths = \
            dubins_path_planning.dubins_path_planning(
            start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)

        check_edge_condition(px, py, pyaw, start_x, start_y, start_yaw, end_x,
                             end_y, end_yaw)
        check_path_length(px, py, lengths)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
