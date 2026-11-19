import conftest
from PathPlanning.AStar import a_star as m


def test_1():
    m.show_animation = False
    m.main()


def test_calc_grid_index_starts_at_zero_for_offset_origin_and_resolution():
    ox = [10.0, 10.0, 20.0, 20.0]
    oy = [-5.0, 5.0, -5.0, 5.0]
    planner = m.AStarPlanner(ox, oy, 0.5, 0.5)

    assert planner.calc_grid_index(planner.Node(0, 0, 0.0, -1)) == 0
    last_index = planner.calc_grid_index(
        planner.Node(planner.x_width - 1, planner.y_width - 1, 0.0, -1))
    assert last_index == planner.x_width * planner.y_width - 1


if __name__ == '__main__':
    conftest.run_this_test(__file__)
