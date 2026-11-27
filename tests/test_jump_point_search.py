import math

import conftest  # noqa  # Add root path to sys.path
from PathPlanning.AStar import a_star
from PathPlanning.JumpPointSearch import jump_point_search as jps

GRID_SIZE = 2.0  # [m]
ROBOT_RADIUS = 1.0  # [m]


def path_length(rx, ry):
    """Total Euclidean length of a cell-by-cell grid path."""
    length = 0.0
    for i in range(len(rx) - 1):
        length += math.hypot(rx[i + 1] - rx[i], ry[i + 1] - ry[i])
    return length


def border_obstacles(lower, upper):
    """Points forming a closed square border from ``lower`` to ``upper``."""
    ox, oy = [], []
    for i in range(lower, upper + 1):
        ox.append(i)
        oy.append(lower)
        ox.append(i)
        oy.append(upper)
        ox.append(lower)
        oy.append(i)
        ox.append(upper)
        oy.append(i)
    return ox, oy


def a_star_module_main_obstacles():
    """The obstacle course used by both a_star.main and jump_point_search.main."""
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)
    return ox, oy


def run_a_star(ox, oy, sx, sy, gx, gy):
    a_star.show_animation = False
    planner = a_star.AStarPlanner(ox, oy, GRID_SIZE, ROBOT_RADIUS)
    return planner.planning(sx, sy, gx, gy)


def run_jps(ox, oy, sx, sy, gx, gy):
    jps.show_animation = False
    planner = jps.JumpPointSearchPlanner(ox, oy, GRID_SIZE, ROBOT_RADIUS)
    return planner.planning(sx, sy, gx, gy)


def assert_matches_a_star(ox, oy, sx, sy, gx, gy):
    """
    Jump Point Search must return a path with the SAME cost as A* because it
    searches the same eight-connected grid. This is the key correctness check.
    """
    a_rx, a_ry = run_a_star(ox, oy, sx, sy, gx, gy)
    j_rx, j_ry = run_jps(ox, oy, sx, sy, gx, gy)

    # Both planners must actually produce a path.
    assert len(a_rx) > 0
    assert len(j_rx) > 0

    # Same start and goal endpoints (output is goal first, start last).
    assert math.isclose(j_rx[0], a_rx[0]) and math.isclose(j_ry[0], a_ry[0])
    assert math.isclose(j_rx[-1], a_rx[-1]) and math.isclose(j_ry[-1], a_ry[-1])

    # Same optimal cost.
    assert math.isclose(path_length(j_rx, j_ry),
                        path_length(a_rx, a_ry),
                        rel_tol=1e-6, abs_tol=1e-9)


def test_optimality_on_main_obstacle_course():
    ox, oy = a_star_module_main_obstacles()
    assert_matches_a_star(ox, oy, 10.0, 10.0, 50.0, 50.0)


def test_optimality_on_open_grid():
    # Only a border: the shortest path is an unobstructed diagonal-plus-straight
    # run, which stresses long diagonal and straight jumps.
    ox, oy = border_obstacles(-10, 60)
    assert_matches_a_star(ox, oy, -5.0, -5.0, 55.0, 30.0)


def test_optimality_with_wall_and_gap():
    # A vertical wall with a gap near the top forces a detour and creates the
    # forced neighbors that Jump Point Search must detect to stay optimal.
    ox, oy = border_obstacles(-10, 60)
    for i in range(-10, 45):
        ox.append(25.0)
        oy.append(i)
    assert_matches_a_star(ox, oy, 0.0, 20.0, 50.0, 20.0)


def test_main():
    # Smoke test: the example runs head-less without raising.
    jps.show_animation = False
    jps.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
