from Mapping.grid_map_lib.grid_map_lib import GridMap
import conftest
import numpy as np


def test_position_set():
    grid_map = GridMap(100, 120, 0.5, 10.0, -0.5)

    grid_map.set_value_from_xy_pos(10.1, -1.1, 1.0)
    grid_map.set_value_from_xy_pos(10.1, -0.1, 1.0)
    grid_map.set_value_from_xy_pos(10.1, 1.1, 1.0)
    grid_map.set_value_from_xy_pos(11.1, 0.1, 1.0)
    grid_map.set_value_from_xy_pos(10.1, 0.1, 1.0)
    grid_map.set_value_from_xy_pos(9.1, 0.1, 1.0)


def test_polygon_set():
    ox = [0.0, 4.35, 20.0, 50.0, 100.0, 130.0, 40.0]
    oy = [0.0, -4.15, -20.0, 0.0, 30.0, 60.0, 80.0]

    grid_map = GridMap(600, 290, 0.7, 60.0, 30.5)

    grid_map.set_value_from_polygon(ox, oy, 1.0, inside=False)
    grid_map.set_value_from_polygon(np.array(ox), np.array(oy),
                                    1.0, inside=False)


def test_xy_and_grid_index_conversion():
    grid_map = GridMap(100, 120, 0.5, 10.0, -0.5)

    for x_ind in range(grid_map.width):
        for y_ind in range(grid_map.height):
            grid_ind = grid_map.calc_grid_index_from_xy_index(x_ind, y_ind)
            x_ind_2, y_ind_2 = grid_map.calc_xy_index_from_grid_index(grid_ind)
            assert x_ind == x_ind_2
            assert y_ind == y_ind_2


def test_world_position_boundaries():
    grid = GridMap(3, 2, 1.0, 1.5, 1.0, init_val=0)
    for x, y in [(0.5, 0.5), (0.5, 1.5), (2.5, 0.5)]:
        assert grid.set_value_from_xy_pos(x, y, 1) is True
    assert grid.data == [1, 0, 1, 1, 0, 0]
    for x, y in [(3, 0.5), (0.5, 2), (-1, 0.5), (1.5, -1)]:
        assert grid.calc_grid_index_from_xy_pos(x, y) is None
        assert grid.set_value_from_xy_pos(x, y, 2) is False
    assert grid.set_value_from_xy_index(None, 0, 2) is False
    assert grid.data == [1, 0, 1, 1, 0, 0]


def test_open_and_closed_polygons_fill_the_same_cells():
    x, y = [0, 0, 3], [0, 3, 3]
    for convert in (list, np.array):
        grid = GridMap(3, 3, 1.0, 1.5, 1.5, init_val=0)
        grid.set_value_from_polygon(convert(x), convert(y), 1)
        closed = GridMap(3, 3, 1.0, 1.5, 1.5, init_val=0)
        closed.set_value_from_polygon(x + x[:1], y + y[:1], 1)
        assert grid.data == closed.data
        assert grid.get_value_from_xy_index(0, 2) == 1
        assert grid.get_value_from_xy_index(2, 0) == 0


if __name__ == '__main__':
    conftest.run_this_test(__file__)
