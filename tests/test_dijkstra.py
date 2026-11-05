import conftest
from PathPlanning.Dijkstra import dijkstra as m


def test_1():
    m.show_animation = False
    m.main()


def test_calc_index_uses_grid_indices():
    # The obstacle map does not start at the origin, so min_x/min_y are
    # non-zero. calc_index must be computed from grid indices only.
    ox = [10.0, 10.0, 20.0, 20.0]
    oy = [-5.0, 5.0, -5.0, 5.0]
    planner = m.DijkstraPlanner(ox, oy, 1.0, 0.5)

    indices = set()
    for ix in range(planner.x_width):
        for iy in range(planner.y_width):
            index = planner.calc_index(planner.Node(ix, iy, 0.0, -1))
            assert 0 <= index < planner.x_width * planner.y_width
            indices.add(index)

    assert len(indices) == planner.x_width * planner.y_width


if __name__ == '__main__':
    conftest.run_this_test(__file__)
