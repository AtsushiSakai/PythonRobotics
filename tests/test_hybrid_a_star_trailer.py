import math

import conftest
import numpy as np
import pytest
from scipy.integrate import solve_ivp
from scipy.spatial import cKDTree

from PathPlanning.HybridAStarTrailer import trailer_hybrid_a_star as m


EMPTY_TREE = cKDTree(np.empty((0, 2)))
BOUNDS = (-100.0, 100.0, -100.0, 100.0)


@pytest.mark.parametrize("distance", [0.2, -0.2])
def test_motion_matches_independent_kinematic_integration(distance):
    pose = [2.0, -3.0, 0.4, -0.2]
    steer = 0.35

    def derivative(_, state):
        return [math.cos(state[2]), math.sin(state[2]),
                math.tan(steer) / m.WHEEL_BASE,
                math.sin(state[2] - state[3]) / m.TRAILER_LENGTH]

    reference = solve_ivp(derivative, (0, distance), pose, rtol=1e-12, atol=1e-13)
    np.testing.assert_allclose(m.move(pose, distance, steer),
                               reference.y[:, -1], atol=1e-9, rtol=0)


@pytest.mark.parametrize("distance", [10.0, -10.0])
def test_straight_aligned_motion(distance):
    pose = [1.0, 2.0, math.pi / 2.0, math.pi / 2.0]
    np.testing.assert_allclose(m.move(pose, distance, 0.0),
                               [1.0, 2.0 + distance, *pose[2:]], atol=1e-12)


def test_forward_stabilizes_and_reverse_amplifies_articulation():
    pose = [0.0, 0.0, 0.0, 0.2]
    assert 0 < m.move(pose, 0.2, 0.0)[3] < pose[3]
    assert m.move(pose, -0.2, 0.0)[3] > pose[3]


@pytest.mark.parametrize("obstacle", [[3.0, 0.0], [-8.0, 0.0], [0.0, m.WIDTH / 2]])
def test_collision_detects_either_body_and_boundary(obstacle):
    assert not m.collision_free([[0, 0, 0, 0]], cKDTree([obstacle]))


def test_collision_uses_trailer_heading():
    pose = [0, 0, 0, math.pi / 2]
    assert not m.collision_free([pose], cKDTree([[0, -8]]))
    assert m.collision_free([pose], cKDTree([[-8, 0]]))
    assert m.collision_free([pose], EMPTY_TREE)


def test_entire_motion_is_checked_for_collision():
    root = m.Node([(0, 0, 0, 0)], [0.0])
    tree = cKDTree([[10.0, 0.0]])
    assert m.collision_free([root.poses[-1], (20, 0, 0, 0)], tree)
    assert m.extend(root, 20.0, 0.0, tree, BOUNDS) is None


def test_reverse_motion_rejects_jackknife():
    root = m.Node([(0, 0, 0, math.radians(70))], [0.0])
    assert m.extend(root, -3.0, 0.0, EMPTY_TREE, BOUNDS) is None


def test_state_key_includes_trailer_and_wraps_angles():
    root = m.Node([(0, 0, math.pi, math.pi)], [0.0])
    wrapped = m.Node([(0, 0, -math.pi, -math.pi)], [0.0])
    different_trailer = m.Node([(0, 0, math.pi, math.pi - 0.5)], [0.0])
    assert m.state_key(root) == m.state_key(wrapped)
    assert m.state_key(root) != m.state_key(different_trailer)


def test_analytic_connection_rejects_wrong_trailer_goal_heading():
    root = m.Node([(0, 0, 0, 0)], [0.0])
    goal = [20, 0, 0, 0.5]
    assert m.analytic_expansion(root, goal, EMPTY_TREE, BOUNDS) is None


def test_singular_tractor_connection_falls_back_to_grid_search():
    radius = m.WHEEL_BASE / math.tan(m.MAX_STEER)
    start, goal = [0, 0, 0, 0], [radius, radius, math.pi / 2, 0.9]
    root = m.Node([start], [0.0])
    assert m.analytic_expansion(root, goal, EMPTY_TREE, BOUNDS) is None
    path = m.hybrid_a_star_planning(start, goal, [], max_expansions=100)
    assert path is not None
    assert path.expanded_nodes > 1
    assert m.at_goal(path.poses[-1], goal)


def test_coincident_tractor_pose_does_not_ignore_trailer_heading():
    root = m.Node([(0, 0, 0, 0)], [0.0])
    goal = [0, 0, 0, 0.5]
    assert not m.at_goal(root.poses[-1], goal)
    assert m.analytic_expansion(root, goal, EMPTY_TREE, BOUNDS) is None


def test_tractor_goal_alone_is_not_success():
    assert not m.at_goal((10, 0, 0, 0.3), (10, 0, 0, 0))
    assert m.at_goal((10, 0, math.pi, -math.pi), (10, 0, -math.pi, math.pi))


@pytest.mark.parametrize("goal_x", [20.0, -20.0])
def test_straight_plan_preserves_inputs_and_reaches_all_four_coordinates(goal_x):
    start, goal, obstacles = [0.0] * 4, [goal_x, 0.0, 0.0, 0.0], []
    original = start[:], goal[:], obstacles[:]
    path = m.hybrid_a_star_planning(start, goal, obstacles)
    assert path is not None
    np.testing.assert_allclose(path.poses[0], start)
    np.testing.assert_allclose(path.poses[-1], goal, atol=1e-10)
    assert np.all(np.sign(path.distances[1:]) == np.sign(goal_x))
    assert (start, goal, obstacles) == original


def test_already_at_goal_and_colliding_start():
    pose = [0, 0, math.pi, -math.pi]
    path = m.hybrid_a_star_planning(pose, pose, [])
    assert path is not None
    assert len(path.poses) == 1
    assert path.cost == 0
    assert m.hybrid_a_star_planning(pose, pose, [[0, 0]]) is None


def test_goal_collision_and_search_limit_return_none():
    start, goal = [0, 0, 0, 0], [25, 0, 0, 0]
    assert m.hybrid_a_star_planning(start, goal, [[25, 0]]) is None
    assert m.hybrid_a_star_planning(start, goal, [], max_expansions=0) is None
    wall = [[10, y] for y in np.arange(-15.0, 15.1, 0.5)]
    assert m.hybrid_a_star_planning(start, goal, wall, bounds=(-10, 40, -5, 5),
                                   max_expansions=100) is None


def test_example_search_is_continuous_feasible_and_reaches_trailer_goal(monkeypatch):
    monkeypatch.setattr(m, "show_animation", False)
    start, goal, obstacles = m.example()
    path = m.main()
    assert path.expanded_nodes > 1  # Exercise the grid search, not just a shortcut.
    np.testing.assert_allclose(path.poses[0], start)
    assert m.at_goal(path.poses[-1], goal)
    assert np.any(path.distances > 0) and np.any(path.distances < 0)
    assert np.max(np.abs(path.distances)) <= m.MOTION_RESOLUTION + 1e-12
    assert np.max(np.abs(path.steers)) <= m.MAX_STEER
    assert m.valid_poses(path.poses, cKDTree(obstacles), BOUNDS)
    for previous, pose, distance, steer in zip(
        path.poses[:-1], path.poses[1:], path.distances[1:], path.steers[1:]
    ):
        np.testing.assert_allclose(m.move(previous, distance, steer), pose,
                                   atol=1e-10, rtol=0)


if __name__ == "__main__":
    conftest.run_this_test(__file__)
