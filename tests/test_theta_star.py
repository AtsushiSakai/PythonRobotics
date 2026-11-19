import math

import conftest
import pytest
from PathPlanning.ThetaStar import theta_star as m


def test_1():
    m.show_animation = False
    m.main()


@pytest.mark.parametrize("min_x,min_y,width,height,resolution", [
    (0, 0, 10, 10, 3),
    (-5, -8, 10, 10, 3),
    (7, 4, 10, 10, 3),
    (0, 0, 10, 12, 3),
    (0, 0, 12, 10, 3),
    (0, 0, 10, 10, 4),
    (0, 0, 10, 10, 1.5),
    (0, 0, 10, 10, 2),
])
def test_non_divisible_grid_extent(min_x, min_y, width, height, resolution):
    m.show_animation = False
    planner = m.ThetaStarPlanner(
        [min_x, min_x + width], [min_y, min_y + height], resolution, 0.1)
    expected_x = len([i for i in range(width + 1) if i * resolution < width])
    expected_y = len([i for i in range(height + 1) if i * resolution < height])
    assert planner.x_width == expected_x
    assert planner.y_width == expected_y
    last_x = min_x + (expected_x - 1) * resolution
    last_y = min_y + (expected_y - 1) * resolution
    inner_x = min_x + resolution
    inner_y = min_y + resolution
    for sx, sy, gx, gy in [(last_x, last_y, inner_x, inner_y),
                          (inner_x, inner_y, last_x, last_y)]:
        rx, ry = planner.planning(sx, sy, gx, gy)
        assert (rx[0], ry[0]) == (gx, gy)
        assert (rx[-1], ry[-1]) == (sx, sy)
        assert all(min_x <= x < min_x + width for x in rx)
        assert all(min_y <= y < min_y + height for y in ry)


def test_last_grid_column_checks_inflated_obstacles():
    planner = m.ThetaStarPlanner([0, 10, 9], [0, 10, 6], 3, 3.1)
    assert not planner.verify_node(planner.Node(3, 2, 0, -1))
    assert not planner.verify_node(planner.Node(3, 1, 0, -1))
    assert planner.verify_node(planner.Node(3, 0, 0, -1))


@pytest.mark.parametrize("resolution", [0.2, math.nextafter(0.2, 0)])
def test_grid_bounds_with_floating_point_resolution(resolution):
    planner = m.ThetaStarPlanner([0, 1], [0, 1], resolution, 0.01)
    for x, y in [(planner.x_width, 1), (1, planner.y_width), (-1, 1), (1, -1)]:
        assert not planner.verify_node(planner.Node(x, y, 0, -1))
    assert planner.verify_node(planner.Node(4, 3, 0, -1))


if __name__ == '__main__':
    conftest.run_this_test(__file__)
