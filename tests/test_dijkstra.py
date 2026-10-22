import conftest
from PathPlanning.Dijkstra import dijkstra as m


def test_1():
    m.show_animation = False
    m.main()


def test_calc_index_uses_grid_coordinates():
    # Grid indices are relative to the map origin and measured in cells.
    for min_x, min_y, resolution in ((-4.0, 3.0, 0.5), (6.0, -5.0, 2.0)):
        ox = [min_x, min_x + 10.0, min_x, min_x + 10.0]
        oy = [min_y, min_y, min_y + 10.0, min_y + 10.0]
        planner = m.DijkstraPlanner(ox, oy, resolution, robot_radius=0.0)

        assert planner.calc_xy_index(min_x + 2 * resolution, min_x) == 2
        assert planner.calc_xy_index(min_y + 3 * resolution, min_y) == 3
        assert planner.calc_index(planner.Node(0, 0, 0.0, -1)) == 0
        assert planner.calc_index(planner.Node(2, 3, 0.0, -1)) == (
            3 * planner.x_width + 2
        )
        assert planner.calc_index(
            planner.Node(planner.x_width - 1, planner.y_width - 1, 0.0, -1)
        ) == planner.x_width * planner.y_width - 1


if __name__ == '__main__':
    conftest.run_this_test(__file__)
