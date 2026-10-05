import conftest  # Add root path to sys.path
import math
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
import numpy as np
from numpy.testing import assert_allclose
import pytest

from PathTracking.vector_pursuit import vector_pursuit as m


def test_straight_target_and_heading_correction_limit():
    assert m.vector_pursuit_curvature(4.0, 0.0, 0.0) == 0.0
    expected = 0.6 / (2.0 * 4.0)  # Paper equation 30: omega / v = theta / (k*d).
    for lateral_offset in (0.0, -1e-12, 1e-12):
        assert_allclose(m.vector_pursuit_curvature(4.0, lateral_offset, 0.6),
                        expected, atol=1e-12)


@pytest.mark.parametrize("angle", [-1.2, -0.2, 0.2, 1.2])
def test_target_tangent_to_circle_reproduces_circle_curvature(angle):
    radius = 8.0
    target_x = radius * math.sin(abs(angle))
    target_y = math.copysign(radius * (1.0 - math.cos(angle)), angle)
    for time_ratio in (0.5, 1.0, 2.0, 10.0):
        curvature = m.vector_pursuit_curvature(target_x, target_y, angle, time_ratio)
        assert_allclose(curvature, math.copysign(1.0 / radius, angle), atol=1e-14)


def test_heading_changes_steering_at_same_target_position():
    # Circle through (0,0) and (2,2) has radius 2 and translation rotation pi/2.
    assert_allclose(m.vector_pursuit_curvature(2.0, 2.0, 0.0), 0.25)
    assert_allclose(m.vector_pursuit_curvature(2.0, 2.0, math.pi / 2), 0.5)
    # Opposing rotational screws cancel, giving a straight command, not infinity.
    assert_allclose(m.vector_pursuit_curvature(2.0, 2.0, -math.pi / 2), 0.0)


def test_mirroring_and_heading_wrap():
    curvature = m.vector_pursuit_curvature(3.0, 1.0, 0.4)
    assert_allclose(m.vector_pursuit_curvature(3.0, -1.0, -0.4), -curvature)
    assert_allclose(m.vector_pursuit_curvature(3.0, 1.0, 0.4 + 2 * math.pi), curvature)
    state = m.State(x=1.0, y=2.0, yaw=math.pi - 0.05)
    target = np.array([-3.0, 2.2, -math.pi + 0.05])
    steering = m.steering_control(state, target)
    target[2] += 2 * math.pi
    assert_allclose(m.steering_control(state, target), steering)


def test_curvature_tends_to_pure_pursuit_when_rotation_correction_is_slow():
    assert_allclose(m.vector_pursuit_curvature(4.0, 1.0, -0.4, time_ratio=1e10),
                    2.0 / 17.0, atol=1e-10)


@pytest.mark.parametrize("target_x, time_ratio", [(0.0, 2.0), (-1.0, 2.0),
                                                 (1.0, 0.0), (1.0, -1.0)])
def test_rejects_nonforward_target_or_nonpositive_time_ratio(target_x, time_ratio):
    with pytest.raises(ValueError):
        m.vector_pursuit_curvature(target_x, 0.0, 0.0, time_ratio)


def test_steering_is_invariant_to_world_translation_and_rotation():
    state = m.State(x=1.0, y=-2.0, yaw=0.3)
    target = np.array([6.0, 0.5, 0.8])
    angle = 1.2
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    translation = np.array([-3.0, 10.0])
    rotated_state = rotation @ [state.x, state.y] + translation
    rotated_target = rotation @ target[:2] + translation
    actual = m.steering_control(m.State(*rotated_state, yaw=state.yaw + angle),
                                [*rotated_target, target[2] + angle])
    assert_allclose(actual, m.steering_control(state, target), atol=1e-14)


def test_steering_limit_and_bicycle_straight_motion():
    state = m.State(y=0.0, speed=2.0)
    assert m.steering_control(state, [0.1, 1.0, math.pi / 2]) == m.MAX_STEER
    state.yaw = math.pi / 2
    state.update(0.0, 0.0)
    assert_allclose([state.x, state.y, state.yaw], [0.0, 2.0 * m.DT, math.pi / 2],
                    atol=1e-14)


def test_target_selection_clamps_endpoint_and_preserves_progress():
    course = np.column_stack((np.arange(11.0), np.zeros((11, 2))))
    target, nearest = m.search_target_index(m.State(x=1.0, y=1.0), course)
    assert (target, nearest) == (5, 1)
    target, nearest = m.search_target_index(m.State(x=9.5, y=0.0), course, 8)
    assert target == 10 and nearest >= 8
    assert m.search_target_index(m.State(x=10.0, y=0.0), course, 10) == (10, 10)
    _, nearest = m.search_target_index(m.State(x=1.0, y=0.0), course, 8)
    assert nearest == 8


@pytest.mark.parametrize("initial_y, initial_yaw", [(-2.0, 0.0), (2.0, 0.2), (-3.0, -0.2)])
def test_s_curve_reaches_goal_with_bounded_controls(initial_y, initial_yaw):
    initial_state = m.State(y=initial_y, yaw=initial_yaw)
    course, history, targets, steering = m.simulate(initial_state=initial_state)
    assert np.isfinite(history).all()
    assert np.linalg.norm(history[-1, 1:3] - course[-1, :2]) <= m.GOAL_TOLERANCE
    assert np.max(abs(steering)) <= m.MAX_STEER
    assert np.all(np.diff(targets) >= 0)
    # Merely selecting the last target must not stop the vehicle early.
    first_goal_target = np.flatnonzero(targets == len(course) - 1)[0]
    assert np.linalg.norm(history[first_goal_target, 1:3] - course[-1, :2]) > 1.0
    assert (initial_state.y, initial_state.yaw, initial_state.speed) == (initial_y, initial_yaw, 0.0)


def test_straight_course_converges_from_lateral_offset():
    course = np.column_stack((np.arange(0.0, 30.1, 0.1), np.zeros((301, 2))))
    _, history, _, _ = m.simulate(course=course)
    assert abs(history[-1, 2]) < 0.05
    assert abs(history[-1, 3]) < 0.02


def test_timeout_is_reported_and_initial_goal_is_complete(tmp_path):
    with pytest.raises(RuntimeError, match="did not reach"):
        m.simulate(max_time=m.DT)
    course = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    result = m.simulate(course, m.State(x=1.0, y=0.0))
    _, history, targets, _ = result
    assert len(history) == 1 and len(targets) == 0
    animation = m.create_animation(*result)
    animation.save(tmp_path / "already_at_goal.gif", writer=PillowWriter(fps=5))
    plt.close("all")


def test_animation_and_headless_main(tmp_path, monkeypatch):
    monkeypatch.setattr(m, "show_animation", False)
    m.main()
    course, history, targets, steering = m.simulate()
    animation = m.create_animation(course, history[::len(history) - 1],
                                   targets[:1], steering[:1])
    path = tmp_path / "vector_pursuit.gif"
    animation.save(path, writer=PillowWriter(fps=5))
    assert path.stat().st_size > 0
    plt.close("all")


if __name__ == "__main__":
    conftest.run_this_test(__file__)
