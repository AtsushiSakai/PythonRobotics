import math

import numpy as np
import pytest

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


def _interpolate_reference(dist, length, mode, max_curvature, origin_x,
                           origin_y, origin_yaw):
    """Scalar interpolation implementation used before vectorization."""
    if mode == "S":
        x = origin_x + dist / max_curvature * math.cos(origin_yaw)
        y = origin_y + dist / max_curvature * math.sin(origin_yaw)
        yaw = origin_yaw
    else:  # curve
        ldx = math.sin(dist) / max_curvature
        ldy = 0.0
        yaw = None
        if mode == "L":  # left turn
            ldy = (1.0 - math.cos(dist)) / max_curvature
            yaw = origin_yaw + dist
        elif mode == "R":  # right turn
            ldy = (1.0 - math.cos(dist)) / -max_curvature
            yaw = origin_yaw - dist
        gdx = math.cos(-origin_yaw) * ldx + math.sin(-origin_yaw) * ldy
        gdy = -math.sin(-origin_yaw) * ldx + math.cos(-origin_yaw) * ldy
        x = origin_x + gdx
        y = origin_y + gdy

    return x, y, yaw, 1 if length > 0.0 else -1


@pytest.mark.parametrize(
    ("mode", "direction"),
    [
        pytest.param("S", 1, id="straight-forward"),
        pytest.param("S", -1, id="straight-reverse"),
        pytest.param("L", 1, id="left-forward"),
        pytest.param("L", -1, id="left-reverse"),
        pytest.param("R", 1, id="right-forward"),
        pytest.param("R", -1, id="right-reverse"),
    ],
)
def test_interpolate_vectorized_matches_scalar_reference(mode, direction):
    rng = np.random.default_rng(1234)

    for _ in range(10):
        length = direction * rng.uniform(0.1, 2.0)
        dists = np.sort(rng.uniform(0.0, abs(length), size=10))
        dists = direction * np.concatenate(([0.0], dists, [abs(length)]))
        max_curvature = rng.uniform(0.1, 2.0)
        origin_x, origin_y = rng.uniform(-10.0, 10.0, size=2)
        origin_yaw = rng.uniform(-np.pi, np.pi)

        expected = [
            _interpolate_reference(
                dist, length, mode, max_curvature, origin_x, origin_y,
                origin_yaw
            )
            for dist in dists
        ]
        expected_x, expected_y, expected_yaw, expected_directions = (
            np.asarray(values) for values in zip(*expected)
        )

        xs, ys, yaws, directions = m.interpolate_vectorized(
            dists, length, mode, max_curvature, origin_x, origin_y,
            origin_yaw
        )

        np.testing.assert_allclose(xs, expected_x)
        np.testing.assert_allclose(ys, expected_y)
        np.testing.assert_allclose(yaws, expected_yaw)
        np.testing.assert_array_equal(directions, expected_directions)


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


if __name__ == '__main__':
    conftest.run_this_test(__file__)
