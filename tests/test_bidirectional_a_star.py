import conftest
from PathPlanning.BidirectionalAStar import bidirectional_a_star as m


def test_1():
    m.show_animation = False
    m.main()


def test_unreachable_goal_returns_empty_path():
    m.show_animation = False
    # Square boundary from 0 to 10 with an inner wall at x = 5.
    ox, oy = [], []
    for i in range(11):
        ox.extend([0.0, 10.0, float(i), float(i), 5.0])
        oy.extend([float(i), float(i), 0.0, 10.0, float(i)])
    planner = m.BidirectionalAStarPlanner(ox, oy, 1.0, 0.5)

    rx, ry = planner.planning(2.0, 5.0, 8.0, 5.0)

    assert rx == []
    assert ry == []


if __name__ == '__main__':
    conftest.run_this_test(__file__)
