import math

import pytest

import conftest
from MissionPlanning.SequentialSingleItemAuction import \
    sequential_single_item_auction as m
from PathPlanning.AStar import a_star
from PathPlanning.Dijkstra import dijkstra


def make_cost_map(walls=(), size=40.0):
    obstacle_x, obstacle_y = m.build_map(size, list(walls))
    return m.TravelCostMap(obstacle_x, obstacle_y, grid_resolution=2.0,
                           robot_radius=1.0)


def test_main_minisum():
    m.show_animation = False
    m.main(m.MINISUM)


def test_main_minimax():
    m.show_animation = False
    m.main(m.MINIMAX)


@pytest.mark.parametrize("objective", [m.MINISUM, m.MINIMAX])
def test_every_target_is_assigned_exactly_once(objective):
    cost_map = make_cost_map()
    robot_starts = [(4.0, 4.0), (36.0, 36.0)]
    targets = [(10.0, 10.0), (30.0, 30.0), (10.0, 30.0), (30.0, 10.0),
               (20.0, 20.0)]
    routes = m.sequential_single_item_auction(robot_starts, targets, cost_map,
                                              objective)
    assigned = [target for route in routes for target in route]
    assert sorted(assigned) == sorted(targets)


def test_single_robot_visits_targets_in_order_along_a_line():
    cost_map = make_cost_map()
    targets = [(30.0, 4.0), (10.0, 4.0), (20.0, 4.0)]
    routes = m.sequential_single_item_auction([(4.0, 4.0)], targets, cost_map)
    assert routes == [[(10.0, 4.0), (20.0, 4.0), (30.0, 4.0)]]


def test_best_insertion_inserts_between_existing_targets():
    cost_map = make_cost_map()
    start = (4.0, 4.0)
    route = [(10.0, 4.0), (30.0, 4.0)]
    insert_index, new_cost = m.best_insertion(cost_map, start, route,
                                              (20.0, 4.0))
    assert insert_index == 1
    assert new_cost == pytest.approx(m.route_cost(cost_map, start, route))


def test_bids_use_path_length_around_obstacles():
    """A target behind a wall goes to the robot with the shorter *path*,
    even though another robot is closer in a straight line."""
    cost_map = make_cost_map(walls=[((20, 0), (20, 34))])
    near_but_blocked = (16.0, 4.0)
    far_but_clear = (34.0, 30.0)
    target = (24.0, 4.0)
    assert (math.dist(near_but_blocked, target)
            < math.dist(far_but_clear, target))
    routes = m.sequential_single_item_auction(
        [near_but_blocked, far_but_clear], [target], cost_map)
    assert routes == [[], [target]]


def test_minisum_and_minimax_trade_off_in_main_scenario():
    m.show_animation = False
    _, minisum_costs = m.main(m.MINISUM)
    _, minimax_costs = m.main(m.MINIMAX)
    assert sum(minisum_costs) < sum(minimax_costs)
    assert max(minimax_costs) < max(minisum_costs)


@pytest.mark.parametrize("wall", [((10.0, 10.0), (10.5, 10.0)),
                                  ((10.0, 10.0), (10.0, 10.0)),
                                  ((10.0, 10.0), (12.5, 10.0))])
def test_build_map_samples_short_and_fractional_walls(wall):
    border_x, _ = m.build_map(40.0, [])
    obstacle_x, obstacle_y = m.build_map(40.0, [wall])
    wall_points = list(zip(obstacle_x, obstacle_y))[len(border_x):]
    assert wall_points[0] == wall[0]
    assert wall_points[-1] == wall[1]
    assert all(math.dist(a, b) <= 1.0
               for a, b in zip(wall_points, wall_points[1:]))


def test_cost_map_leaves_planner_animation_settings_alone(monkeypatch):
    monkeypatch.setattr(a_star, "show_animation", True)
    monkeypatch.setattr(dijkstra, "show_animation", True)
    cost_map = make_cost_map()
    cost_map.cost((4.0, 4.0), (30.0, 30.0))
    assert a_star.show_animation is True
    assert dijkstra.show_animation is True


def test_one_search_per_robot_start_and_target():
    cost_map = make_cost_map(walls=[((20, 0), (20, 34))])
    robot_starts = [(4.0, 4.0), (36.0, 36.0)]
    targets = [(10.0, 10.0), (30.0, 30.0), (10.0, 30.0), (30.0, 10.0),
               (24.0, 4.0)]
    m.sequential_single_item_auction(robot_starts, targets, cost_map)
    assert len(cost_map.searches) == len(robot_starts) + len(targets)


def test_path_lengths_match_a_star(monkeypatch):
    walls = [((20, 0), (20, 34))]
    cost_map = make_cost_map(walls=walls)
    obstacle_x, obstacle_y = m.build_map(40.0, walls)
    monkeypatch.setattr(a_star, "show_animation", False)
    planner = a_star.AStarPlanner(obstacle_x, obstacle_y, 2.0, 1.0)
    for start, goal in [((4.0, 4.0), (30.0, 4.0)), ((16.0, 4.0), (24.0, 4.0)),
                        ((36.0, 36.0), (10.0, 10.0))]:
        path_x, path_y = planner.planning(*start, *goal)
        a_star_length = sum(math.hypot(x1 - x0, y1 - y0) for x0, y0, x1, y1
                            in zip(path_x, path_y, path_x[1:], path_y[1:]))
        path_x, path_y, length = cost_map.path(start, goal)
        assert length == pytest.approx(a_star_length)
        assert (path_x[0], path_y[0]) == start
        assert (path_x[-1], path_y[-1]) == goal


def test_unreachable_target_raises():
    # a closed box around (30, 30)
    walls = [((24, 24), (36, 24)), ((36, 24), (36, 36)),
             ((24, 36), (36, 36)), ((24, 24), (24, 36))]
    cost_map = make_cost_map(walls=walls)
    with pytest.raises(ValueError):
        cost_map.cost((4.0, 4.0), (30.0, 30.0))


def test_unknown_objective_raises():
    with pytest.raises(ValueError):
        m.calc_bid("fastest", 1.0, 2.0)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
