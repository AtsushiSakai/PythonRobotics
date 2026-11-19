"""Check complete arm links against sphere obstacles with real DH kinematics."""
import conftest  # Add root path to sys.path
import numpy as np
import pytest

from ArmNavigation.rrt_star_seven_joint_arm_control \
    import rrt_star_seven_joint_arm_control as m


@pytest.mark.parametrize(
    ("center", "radius", "safe"),
    [
        ((0.5, 0.0, 0.0), 0.1, False),
        ((0.5, 0.1, 0.0), 0.1, False),
        ((0.5, 0.10001, 0.0), 0.1, True),
        ((0.5, 0.0, 0.1), 0.1, False),
        ((0.5, 0.0, 0.10001), 0.1, True),
        ((-0.5, 0.0, 0.0), 0.1, True),
        ((1.5, 0.0, 0.0), 0.1, True),
        ((0.0, 0.0, 0.0), 0.05, False),
        ((1.0, 0.0, 0.0), 0.05, False),
    ],
)
def test_link_interior_tangency_and_segment_limits(center, radius, safe):
    robot = m.RobotArm([[0.0, 0.0, 1.0, 0.0]])
    node = m.RRTStar.Node([0.0])
    node.path_x = [[0.0]]
    assert m.RRTStar.check_collision(node, robot, [(*center, radius)]) == safe


def test_rotated_link_uses_kinematic_endpoints():
    robot = m.RobotArm([[0.0, 0.0, 1.0, 0.0]])
    node = m.RRTStar.Node([np.pi / 2])
    node.path_x = [[np.pi / 2]]
    assert not m.RRTStar.check_collision(node, robot, [(0.0, 0.5, 0.0, 0.1)])
    assert m.RRTStar.check_collision(node, robot, [(0.5, 0.0, 0.0, 0.1)])


def test_link_interior_in_vertical_three_dimensional_arm():
    robot = m.RobotArm([[0.0, 0.0, 0.0, 1.0]])
    node = m.RRTStar.Node([0.0])
    node.path_x = [[0.0]]
    assert not m.RRTStar.check_collision(node, robot, [(0.0, 0.0, 0.5, 0.1)])


def test_later_link_of_bent_arm_is_checked():
    robot = m.RobotArm([[0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 1.0, 0.0]])
    node = m.RRTStar.Node([0.0, np.pi / 2])
    node.path_x = [[0.0, np.pi / 2]]
    assert not m.RRTStar.check_collision(node, robot, [(1.0, 0.5, 0.0, 0.1)])


def test_all_sampled_joint_configurations_are_checked():
    robot = m.RobotArm([[0.0, 0.0, 1.0, 0.0]])
    node = m.RRTStar.Node([0.0])
    node.path_x = [[np.pi / 2], [0.0]]
    assert not m.RRTStar.check_collision(node, robot, [(0.5, 0.0, 0.0, 0.1)])


def test_zero_length_links_and_linkless_base_remain_finite():
    for dh_params, angles in [([[0.0, 0.0, 0.0, 0.0]], [0.0]), ([], [])]:
        robot = m.RobotArm(dh_params)
        node = m.RRTStar.Node(angles)
        node.path_x = [angles]
        with np.errstate(all="raise"):
            assert not m.RRTStar.check_collision(node, robot, [(0.0, 0.0, 0.0, 0.1)])
            assert m.RRTStar.check_collision(node, robot, [(1.0, 0.0, 0.0, 0.1)])


def test_seven_joint_panda_first_link_interior_collision():
    robot = m.RobotArm([
        [0.0, np.pi / 2, 0.0, 0.333],
        [0.0, -np.pi / 2, 0.0, 0.0],
        [0.0, np.pi / 2, 0.0825, 0.3160],
        [0.0, -np.pi / 2, -0.0825, 0.0],
        [0.0, np.pi / 2, 0.0, 0.3840],
        [0.0, np.pi / 2, 0.088, 0.0],
        [0.0, 0.0, 0.0, 0.107],
    ])
    node = m.RRTStar.Node([0.0] * 7)
    node.path_x = [[0.0] * 7]
    assert not m.RRTStar.check_collision(node, robot, [(0.0, 0.0, 0.1665, 0.02)])


if __name__ == "__main__":
    conftest.run_this_test(__file__)
