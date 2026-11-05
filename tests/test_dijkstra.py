import conftest
from PathPlanning.Dijkstra import dijkstra as m


def test_1():
    m.show_animation = False
    m.main()


def test_calc_index_starts_at_zero_for_offset_origin_and_resolution():
    # Map origin is not (0, 0) and the resolution is not 1, so mixing the
    # origin in meters into the grid index would shift the indices.
    ox = [10.0, 10.0, 20.0, 20.0]
    oy = [-5.0, 5.0, -5.0, 5.0]
    planner = m.DijkstraPlanner(ox, oy, 0.5, 0.5)

    assert planner.calc_index(planner.Node(0, 0, 0.0, -1)) == 0

    indices = set()
    for ix in range(planner.x_width):
        for iy in range(planner.y_width):
            index = planner.calc_index(planner.Node(ix, iy, 0.0, -1))
            assert 0 <= index < planner.x_width * planner.y_width
            indices.add(index)

    assert len(indices) == planner.x_width * planner.y_width


if __name__ == '__main__':
    conftest.run_this_test(__file__)
