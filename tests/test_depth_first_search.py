import conftest
from PathPlanning.DepthFirstSearch import depth_first_search as m


def test_1():
    m.show_animation = False
    m.main()



def make_walled_map():
    """Square boundary from 0 to 10 with an inner wall at x = 5."""
    ox, oy = [], []
    for i in range(11):
        ox.extend([0.0, 10.0, float(i), float(i), 5.0])
        oy.extend([float(i), float(i), 0.0, 10.0, float(i)])
    return ox, oy


def test_unreachable_goal_returns_empty_path():
    m.show_animation = False
    ox, oy = make_walled_map()
    planner = m.DepthFirstSearchPlanner(ox, oy, 1.0, 0.5)

    rx, ry = planner.planning(2.0, 5.0, 8.0, 5.0)

    assert rx == []
    assert ry == []


def test_start_equal_to_goal_returns_single_point_path():
    m.show_animation = False
    ox, oy = make_walled_map()
    planner = m.DepthFirstSearchPlanner(ox, oy, 1.0, 0.5)

    rx, ry = planner.planning(2.0, 5.0, 2.0, 5.0)

    assert rx == [2.0]
    assert ry == [5.0]


if __name__ == '__main__':
    conftest.run_this_test(__file__)
