from Mapping.grid_map_lib.grid_map_lib import FloatGrid, GridMap
import conftest
import numpy as np
import pytest


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


@pytest.mark.parametrize("x_ind, y_ind", [
    (-1, 1), (3, 0), (-3, 1), (6, 0), (3, -1), (-1, 2),
    (0, -1), (0, 2),
])
def test_out_of_bounds_indices_do_not_alias_valid_cells(x_ind, y_ind):
    grid_map = GridMap(3, 2, 0.5, 10.0, -0.5)
    original = grid_map.data.copy()

    assert grid_map.get_value_from_xy_index(x_ind, y_ind) is None
    assert grid_map.check_occupied_from_xy_index(x_ind, y_ind, FloatGrid(1.0))
    assert grid_map.set_value_from_xy_index(x_ind, y_ind, FloatGrid(1.0)) is False
    assert grid_map.data == original


@pytest.mark.parametrize("index_type", [int, float, np.int64])
def test_valid_cell_indices_preserve_row_major_storage(index_type):
    grid_map = GridMap(3, 2, 0.5, 10.0, -0.5)

    for y_ind in range(grid_map.height):
        for x_ind in range(grid_map.width):
            value = FloatGrid(y_ind * grid_map.width + x_ind + 1.0)
            assert grid_map.set_value_from_xy_index(
                index_type(x_ind), index_type(y_ind), value) is True
            assert grid_map.get_value_from_xy_index(
                index_type(x_ind), index_type(y_ind)) is value

    assert [value.get_float_data() for value in grid_map.data] == list(range(1, 7))


@pytest.mark.parametrize("obstacle, expected", [
    ((2, 0), {(2, 0), (1, 0), (2, 1)}),
    ((0, 2), {(0, 2), (1, 2), (0, 1)}),
])
def test_expand_grid_does_not_wrap_across_edges(obstacle, expected):
    grid_map = GridMap(3, 3, 1.0, 0.0, 0.0)
    grid_map.set_value_from_xy_index(*obstacle, FloatGrid(1.0))

    grid_map.expand_grid()

    occupied = {
        (x_ind, y_ind)
        for y_ind in range(grid_map.height)
        for x_ind in range(grid_map.width)
        if grid_map.check_occupied_from_xy_index(x_ind, y_ind, FloatGrid(1.0))
    }
    assert occupied == expected


if __name__ == '__main__':
    conftest.run_this_test(__file__)
