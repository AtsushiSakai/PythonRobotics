import conftest
from PathPlanning.BidirectionalBreadthFirstSearch import (
    bidirectional_breadth_first_search as m,
)


def make_planner(blocked=False):
    ox, oy = [], []
    for i in range(8):
        ox.extend([i, i, 0, 7])
        oy.extend([0, 7, i, i])
    if blocked:
        for y in range(1, 7):
            ox.append(3)
            oy.append(y)
    return m.BidirectionalBreadthFirstSearchPlanner(ox, oy, 1.0, 0.0)


def test_unreachable_goal_returns_empty_path():
    m.show_animation = False
    planner = make_planner(blocked=True)

    rx, ry = planner.planning(1.0, 2.0, 5.0, 2.0)

    assert rx == []
    assert ry == []


def test_adjacent_goal_has_single_meeting_cell():
    m.show_animation = False
    planner = make_planner()

    rx, ry = planner.planning(1.0, 2.0, 2.0, 2.0)

    assert list(zip(rx, ry)) == [(1.0, 2.0), (2.0, 2.0)]


def test_coincident_start_and_goal_has_one_waypoint():
    m.show_animation = False
    planner = make_planner()

    rx, ry = planner.planning(1.0, 2.0, 1.0, 2.0)

    assert list(zip(rx, ry)) == [(1.0, 2.0)]


def test_reachable_path_is_contiguous_without_duplicate_meeting():
    m.show_animation = False
    planner = make_planner()

    rx, ry = planner.planning(1.0, 1.0, 5.0, 5.0)
    path = list(zip(rx, ry))

    assert path[0] == (1.0, 1.0)
    assert path[-1] == (5.0, 5.0)
    assert len(path) > 2
    assert all(
        max(abs(ax - bx), abs(ay - by)) == 1.0
        for (ax, ay), (bx, by) in zip(path, path[1:])
    )


if __name__ == '__main__':
    conftest.run_this_test(__file__)
