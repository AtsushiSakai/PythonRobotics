"""Select the cheapest complete joint-space path, including the goal edge."""
import conftest  # Add root path to sys.path
import numpy as np
import pytest

from ArmNavigation.rrt_star_seven_joint_arm_control \
    import rrt_star_seven_joint_arm_control as m


def make_planner(goal=(1.0, 1.0, 0.0)):
    m.show_animation = False
    return m.RRTStar(
        [0.0, 0.0, 0.0], list(goal),
        m.RobotArm([[0.0, 0.0, 1.0, 0.0] for _ in range(3)]), [],
        [-1.0, 1.0], path_resolution=0.1, expand_dis=0.3,
    )


def make_competing_branches(detour=(0.7, 0.4, 0.0), far=(0.8, 0.8, 0.0)):
    planner = make_planner()
    detour_node = m.RRTStar.Node(list(detour))
    detour_node.parent = planner.start
    detour_node.cost = planner.calc_new_cost(planner.start, detour_node)
    far_node = m.RRTStar.Node(list(far))
    far_node.parent = detour_node
    far_node.cost = planner.calc_new_cost(detour_node, far_node)
    close_node = m.RRTStar.Node([1.0, 0.95, 0.0])
    close_node.parent = planner.start
    close_node.cost = planner.calc_new_cost(planner.start, close_node)
    planner.node_list = [planner.start, detour_node, far_node, close_node]
    return planner, far_node, close_node


@pytest.mark.parametrize(("detour", "far"), [
    ((0.7, 0.4, 0.0), (0.8, 0.8, 0.0)),
    ((0.8, 0.6, 0.0), (0.9, 0.9, 0.0)),
])
def test_select_minimum_complete_cost_with_real_parent_branches(detour, far):
    planner, far_node, close_node = make_competing_branches(detour, far)
    assert far_node.cost < close_node.cost
    far_total = planner.calc_new_cost(far_node, planner.goal_node)
    close_total = planner.calc_new_cost(close_node, planner.goal_node)
    assert close_total < far_total
    assert planner.search_best_goal_node() == 3
    path = np.array(planner.generate_final_course(3))
    assert np.linalg.norm(np.diff(path, axis=0), axis=1).sum() == pytest.approx(close_total)


def test_cheapest_total_cost_edge_still_requires_collision_clearance():
    planner, _, close_node = make_competing_branches()
    x, y, z = planner.robot.get_points(close_node.x)
    # Hit the second joint of the direct branch's starting configuration.
    planner.obstacle_list = [(x[2], y[2], z[2], 0.0001)]
    assert not planner.check_collision(
        planner.steer(close_node, planner.goal_node), planner.robot, planner.obstacle_list,
    )
    assert planner.search_best_goal_node() == 2


def test_no_collision_free_goal_connection_returns_none():
    planner, far_node, close_node = make_competing_branches()
    obstacles = []
    for node in (far_node, close_node):
        x, y, z = planner.robot.get_points(node.x)
        obstacles.append((x[2], y[2], z[2], 0.0001))
    planner.obstacle_list = obstacles
    assert planner.search_best_goal_node() is None


def test_nodes_outside_goal_connection_radius_return_none():
    planner = make_planner()
    planner.node_list = [planner.start]
    assert planner.search_best_goal_node() is None


def test_equal_complete_cost_preserves_first_candidate():
    planner = make_planner(goal=(1.0, 0.0, 0.0))
    first = m.RRTStar.Node([0.75, 0.0, 0.0])
    second = m.RRTStar.Node([0.875, 0.0, 0.0])
    for node in (first, second):
        node.parent = planner.start
        node.cost = planner.calc_new_cost(planner.start, node)
    planner.node_list = [planner.start, first, second]
    assert planner.calc_new_cost(first, planner.goal_node) == 1.0
    assert planner.calc_new_cost(second, planner.goal_node) == 1.0
    assert planner.search_best_goal_node() == 1


if __name__ == "__main__":
    conftest.run_this_test(__file__)
