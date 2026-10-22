import conftest
from PathPlanning.Dijkstra import dijkstra as m


def test_1():
    m.show_animation = False
    m.main()


def test_unreachable_goal():
    m.show_animation = False

    ox, oy = [], []
    for i in range(7):
        ox.extend([0.0, 6.0, float(i), float(i), 3.0])
        oy.extend([float(i), float(i), 0.0, 6.0, float(i)])

    planner = m.DijkstraPlanner(ox, oy, 1.0, 0.0)
    rx, ry = planner.planning(1.0, 3.0, 5.0, 3.0)

    assert rx == []
    assert ry == []


if __name__ == '__main__':
    conftest.run_this_test(__file__)
