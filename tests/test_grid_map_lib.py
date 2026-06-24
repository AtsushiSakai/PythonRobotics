from Mapping.grid_map_lib.grid_map_lib import GridMap, FloatGrid
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


def test_set_xy_pos_at_origin_index():
    # Regression: the guard used to be `if (not x_ind) or (not y_ind):`
    # which rejected the valid grid index 0 (since `not 0` is True in
    # Python). Setting a value at the (0, 0) cell must succeed and the
    # return value must be a plain `True` (not the tuple `(False, False)`
    # that the previous None-guard returned).
    grid_map = GridMap(4, 4, 1.0, 2.0, 2.0)  # left_lower = (0, 0)
    ok = grid_map.set_value_from_xy_index(0, 0, FloatGrid(0.5))
    assert ok is True
    assert grid_map.data[0].get_float_data() == 0.5

    # And the higher-level pos API should accept the (0, 0) cell.
    ok = grid_map.set_value_from_xy_pos(0.5, 0.5, FloatGrid(0.7))
    assert ok is True
    assert grid_map.data[0].get_float_data() == 0.7


if __name__ == '__main__':
    conftest.run_this_test(__file__)
