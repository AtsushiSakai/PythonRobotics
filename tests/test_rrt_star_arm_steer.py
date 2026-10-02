"""Keep RRT* arm vertices consistent with their sampled edge endpoints."""
import conftest  # Add root path to sys.path
import numpy as np
import pytest

from ArmNavigation.rrt_star_seven_joint_arm_control \
    import rrt_star_seven_joint_arm_control as m


def make_planner():
    m.show_animation = False
    return m.RRTStar(
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
        m.RobotArm([[0.0, 0.0, 1.0, 0.0] for _ in range(3)]), [], [-1.0, 1.0],
        path_resolution=0.1,
    )


@pytest.mark.parametrize("target", [
    [0.05, 0.0, 0.0],
    [0.25, 0.0, 0.0],
    [0.15, 0.2, 0.0],
    [0.0, 0.0, 0.35],
])
def test_finite_remainder_vertex_matches_target_and_path_endpoint(target):
    planner = make_planner()
    goal = m.RRTStar.Node(target)
    node = planner.steer(planner.start, goal)
    np.testing.assert_allclose(node.x, target)
    np.testing.assert_allclose(node.path_x[-1], node.x)
    assert node.parent is planner.start
    np.testing.assert_array_equal(planner.start.x, [0.0, 0.0, 0.0])


def test_bounded_extension_vertex_matches_last_sample():
    planner = make_planner()
    goal = m.RRTStar.Node([1.0, 0.0, 0.0])
    node = planner.steer(planner.start, goal, extend_length=0.25)
    np.testing.assert_allclose(node.x, [0.2, 0.0, 0.0])
    np.testing.assert_array_equal(node.path_x[-1], node.x)


def test_identical_vertices_do_not_divide_by_zero():
    planner = make_planner()
    goal = m.RRTStar.Node(list(planner.start.x))
    with np.errstate(all="raise"):
        node = planner.steer(planner.start, goal)
    np.testing.assert_array_equal(node.x, planner.start.x)
    np.testing.assert_array_equal(node.path_x[-1], node.x)
    assert node.parent is planner.start


def test_choose_parent_cost_is_for_the_returned_vertex():
    planner = make_planner()
    planner.node_list = [planner.start]
    new_node = m.RRTStar.Node([0.05, 0.0, 0.0])
    selected = planner.choose_parent(new_node, [0])
    np.testing.assert_array_equal(selected.x, new_node.x)
    np.testing.assert_array_equal(selected.path_x[-1], selected.x)
    assert selected.cost == pytest.approx(planner.calc_new_cost(planner.start, selected))


def test_successive_edges_start_at_the_previous_checked_endpoint():
    planner = make_planner()
    first = planner.steer(planner.start, m.RRTStar.Node([0.25, 0.0, 0.0]))
    second = planner.steer(first, m.RRTStar.Node([0.45, 0.0, 0.0]))
    np.testing.assert_array_equal(second.path_x[0], first.path_x[-1])
    np.testing.assert_allclose(second.x, [0.45, 0.0, 0.0])


if __name__ == "__main__":
    conftest.run_this_test(__file__)
