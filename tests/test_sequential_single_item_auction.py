import math

import pytest

import conftest
from MissionPlanning.SequentialSingleItemAuction import \
    sequential_single_item_auction as m


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


def test_unknown_objective_raises():
    with pytest.raises(ValueError):
        m.calc_bid("fastest", 1.0, 2.0)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
