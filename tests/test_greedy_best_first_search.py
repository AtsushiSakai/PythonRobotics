import conftest
from PathPlanning.GreedyBestFirstSearch import greedy_best_first_search as m


def test_1():
    m.show_animation = False
    m.main()


def _make_bounded_map(block_center=False):
    ox, oy = [], []
    for i in range(8):
        ox.extend([i, i, 0, 7])
        oy.extend([0, 7, i, i])
    if block_center:
        for y in range(1, 7):
            ox.append(3)
            oy.append(y)
    return ox, oy


def test_unreachable_goal_returns_empty_path():
    m.show_animation = False
    ox, oy = _make_bounded_map(block_center=True)
    planner = m.BestFirstSearchPlanner(ox, oy, 1.0, 0.0)

    rx, ry = planner.planning(1.0, 2.0, 5.0, 2.0)

    assert rx == []
    assert ry == []


def test_adjacent_goal_appears_only_once():
    m.show_animation = False
    ox, oy = _make_bounded_map()
    planner = m.BestFirstSearchPlanner(ox, oy, 1.0, 0.0)

    rx, ry = planner.planning(1.0, 2.0, 2.0, 2.0)

    assert list(zip(rx, ry)) == [(2.0, 2.0), (1.0, 2.0)]


def test_start_equals_goal_returns_single_point():
    m.show_animation = False
    ox, oy = _make_bounded_map()
    planner = m.BestFirstSearchPlanner(ox, oy, 1.0, 0.0)

    rx, ry = planner.planning(1.0, 2.0, 1.0, 2.0)

    assert list(zip(rx, ry)) == [(1.0, 2.0)]


if __name__ == '__main__':
    conftest.run_this_test(__file__)
