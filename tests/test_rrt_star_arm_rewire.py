"""Rewiring must keep descendant edges connected to the updated tree."""
import conftest  # Add root path to sys.path
import numpy as np
import pytest

from ArmNavigation.rrt_star_seven_joint_arm_control \
    import rrt_star_seven_joint_arm_control as m


def make_tree(obstacles=None, near_position=0.2):
    m.show_animation = False
    planner = m.RRTStar(
        [0.0, 0.0, 0.0], [0.5, 0.0, 0.0],
        m.RobotArm([[0.0, 0.0, 1.0, 0.0] for _ in range(3)]),
        [] if obstacles is None else obstacles, [-1.0, 1.0],
        path_resolution=0.1,
    )
    detour = m.RRTStar.Node([0.0, 1.0, 0.0])
    detour.parent = planner.start
    detour.cost = planner.calc_new_cost(planner.start, detour)
    new = m.RRTStar.Node([0.1, 0.0, 0.0])
    new.parent = planner.start
    new.cost = planner.calc_new_cost(planner.start, new)
    new.path_x = [list(planner.start.x), list(new.x)]
    near = m.RRTStar.Node([near_position, 0.0, 0.0])
    near.parent = detour
    near.cost = planner.calc_new_cost(detour, near)
    near.path_x = [list(detour.x), list(near.x)]
    child = m.RRTStar.Node([near_position + 0.1, 0.0, 0.0])
    child.parent = near
    child.cost = planner.calc_new_cost(near, child)
    child.path_x = [list(near.x), list(child.x)]
    grandchild = m.RRTStar.Node([near_position + 0.2, 0.0, 0.0])
    grandchild.parent = child
    grandchild.cost = planner.calc_new_cost(child, grandchild)
    grandchild.path_x = [list(child.x), list(grandchild.x)]
    planner.node_list = [new, near, child, grandchild, planner.start, detour]
    return planner, new, near, child, grandchild


def test_rewire_preserves_vertex_referenced_by_descendants():
    planner, new, near, child, _ = make_tree()
    planner.rewire(new, [1])
    assert planner.node_list[1] is near
    assert child.parent is planner.node_list[1]
    assert near.parent is new
    assert near.cost == pytest.approx(0.2)
    np.testing.assert_allclose(near.path_x[0], new.x)
    np.testing.assert_allclose(near.path_x[-1], near.x)


def test_rewire_propagates_cost_and_keeps_final_course_on_updated_tree():
    planner, new, near, child, grandchild = make_tree()
    planner.rewire(new, [1])
    assert child.cost == pytest.approx(0.3)
    assert grandchild.cost == pytest.approx(0.4)
    for node in (near, child, grandchild):
        assert any(node.parent is vertex for vertex in planner.node_list)
        assert node.cost == pytest.approx(planner.calc_new_cost(node.parent, node))
    expected_path = [planner.end.x, grandchild.x, child.x, near.x, new.x, planner.start.x]
    np.testing.assert_allclose(planner.generate_final_course(3), expected_path)


def test_non_improving_connection_leaves_tree_unchanged():
    planner, new, near, child, grandchild = make_tree()
    near.parent = planner.start
    near.cost = planner.calc_new_cost(planner.start, near)
    child.cost = planner.calc_new_cost(near, child)
    grandchild.cost = planner.calc_new_cost(child, grandchild)
    old_parent = near.parent
    planner.rewire(new, [1])
    assert planner.node_list[1] is near
    assert near.parent is old_parent
    assert near.cost == pytest.approx(0.2)
    assert child.parent is near
    assert child.cost == pytest.approx(0.3)


def test_rewire_preserves_off_grid_position_and_descendant_edge_anchors():
    planner, new, near, child, grandchild = make_tree(near_position=0.25)
    original_position = list(near.x)
    planner.rewire(new, [1])
    assert planner.node_list[1] is near
    np.testing.assert_array_equal(near.x, original_position)
    np.testing.assert_allclose(near.path_x[-1], near.x)
    np.testing.assert_array_equal(child.path_x[0], near.x)
    np.testing.assert_array_equal(grandchild.path_x[0], child.x)
    assert near.cost == pytest.approx(0.25)
    assert child.cost == pytest.approx(0.35)
    assert grandchild.cost == pytest.approx(0.45)


def test_colliding_connection_leaves_tree_unchanged():
    # The first joint endpoint at q=0.2 lies at this sphere center.
    planner, new, near, child, _ = make_tree([
        (np.cos(0.2), np.sin(0.2), 0.0, 0.01),
    ])
    old_parent = near.parent
    old_cost = near.cost
    old_child_cost = child.cost
    planner.rewire(new, [1])
    assert planner.node_list[1] is near
    assert near.parent is old_parent
    assert near.cost == old_cost
    assert child.parent is near
    assert child.cost == old_child_cost


if __name__ == "__main__":
    conftest.run_this_test(__file__)
