import conftest
import numpy as np
import numpy.testing as nt
import pytest
from PathPlanning.ClothoidPath import clothoid_path_planner as m


def test_1():
    m.show_animation = False
    m.main()


@pytest.mark.parametrize("angle", [0.0, 0.6, -1.0])
def test_scalar_integral_controls(angle: float) -> None:
    nt.assert_allclose(m.X(0.0, 0.0, angle), np.cos(angle), atol=1e-12)
    nt.assert_allclose(m.Y(0.0, 0.0, angle), np.sin(angle), atol=1e-12)


@pytest.mark.parametrize("turn", [-1.0, 1.0])
def test_scalar_circle_parameter_controls(turn: float) -> None:
    radius = 2.0
    delta = turn * np.pi / 2.0
    length = m.compute_path_length(np.sqrt(2.0) * radius, -delta / 2.0, delta, 0.0)
    nt.assert_allclose(length, radius * np.pi / 2.0, atol=1e-12)
    nt.assert_allclose(m.compute_curvature(delta, 0.0, length), turn / radius)
    nt.assert_allclose(m.compute_curvature_rate(0.0, length), 0.0)


@pytest.mark.parametrize("length", [0.5, 2.0])
@pytest.mark.parametrize("angle", [0.0, 0.7, -2.4])
def test_straight_path_coordinates(length: float, angle: float) -> None:
    start = np.array([-1.5, 0.4])
    direction = np.array([np.cos(angle), np.sin(angle)])
    goal = start + length * direction
    path = m.generate_clothoid_path(m.Point(*start), angle, m.Point(*goal), angle, 17)

    assert path is not None
    expected = start + np.linspace(0.0, length, 17)[:, None] * direction
    nt.assert_allclose(np.array(path), expected, atol=1e-12)


@pytest.mark.parametrize("radius", [0.5, 2.0])
@pytest.mark.parametrize("turn", [-1.0, 1.0])
@pytest.mark.parametrize("rotation", [0.0, 0.7])
def test_circle_path_coordinates(radius: float, turn: float, rotation: float) -> None:
    start = np.array([-1.5, 0.4])
    angle = np.linspace(0.0, np.pi / 2.0, 17)
    local = radius * np.column_stack((np.sin(angle), turn * (1.0 - np.cos(angle))))
    transform = np.array(
        [[np.cos(rotation), -np.sin(rotation)], [np.sin(rotation), np.cos(rotation)]]
    )
    expected = start + local @ transform.T
    path = m.generate_clothoid_path(
        m.Point(*start), rotation, m.Point(*expected[-1]), rotation + turn * np.pi / 2, 17
    )

    assert path is not None
    nt.assert_allclose(np.array(path), expected, atol=1e-12)


def test_multiple_orientation_paths() -> None:
    start = m.Point(-1.5, 0.4)
    goal = m.Point(8.0, 2.0)
    paths = m.generate_clothoid_paths(start, [-0.6, 0.3], goal, [-0.4, 0.8], 25)

    assert len(paths) == 4
    for path in paths:
        assert path is not None
        coordinates = np.array(path)
        assert coordinates.shape == (25, 2)
        assert np.isfinite(coordinates).all()
        nt.assert_allclose(coordinates[0], start, atol=1e-10)
        nt.assert_allclose(coordinates[-1], goal, atol=1e-10)


def test_example_orientation_grid_has_complete_paths() -> None:
    start = m.Point(0.0, 0.0)
    goal = m.Point(10.0, 0.0)
    paths = m.generate_clothoid_paths(start, [0.0], goal, np.linspace(-np.pi, np.pi, 75), 100)

    assert len(paths) == 75
    for path in paths:
        assert path is not None
        coordinates = np.array(path)
        assert coordinates.shape == (100, 2)
        assert np.isfinite(coordinates).all()
        nt.assert_allclose(coordinates[0], start, atol=1e-10)
        nt.assert_allclose(coordinates[-1], goal, atol=1e-10)


@pytest.mark.parametrize("invalid_yaw", [float("nan"), float("inf")])
@pytest.mark.parametrize("invalid_start", [False, True])
def test_nonfinite_orientation_has_no_path(invalid_yaw: float, invalid_start: bool) -> None:
    start_yaw = invalid_yaw if invalid_start else 0.0
    goal_yaw = 0.0 if invalid_start else invalid_yaw

    path = m.generate_clothoid_path(m.Point(0.0, 0.0), start_yaw,
                                   m.Point(2.0, 0.0), goal_yaw, 17)

    assert path is None


def test_zero_length_segment_has_no_path() -> None:
    point = m.Point(0.0, 0.0)

    assert m.generate_clothoid_path(point, 0.0, point, 0.0, 17) is None


if __name__ == '__main__':
    conftest.run_this_test(__file__)
