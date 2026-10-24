import conftest
import numpy as np
import pytest

from SLAM.ICPMatching import icp_matching as m


def test_1():
    m.show_animation = False
    m.main()


def test_2():
    m.show_animation = False
    m.main_3d_points()


def rotation_matrix(dimension, angle):
    rotation = np.eye(dimension)
    rotation[:2, :2] = [[np.cos(angle), -np.sin(angle)],
                       [np.sin(angle), np.cos(angle)]]
    return rotation


@pytest.mark.parametrize("dimension", [2, 3])
def test_first_homogeneous_update(dimension):
    rotation = rotation_matrix(dimension, 0.3)
    translation = np.arange(1, dimension + 1, dtype=float)

    transform = m.update_homogeneous_matrix(None, rotation, translation)

    np.testing.assert_allclose(transform[:-1, :-1], rotation)
    np.testing.assert_allclose(transform[:-1, -1], translation)
    np.testing.assert_array_equal(transform[-1], np.r_[np.zeros(dimension), 1.0])


@pytest.mark.parametrize("dimension", [2, 3])
def test_homogeneous_updates_match_sequential_point_transforms(dimension):
    points = np.arange(dimension * 4, dtype=float).reshape(dimension, 4)
    transformed_points = points.copy()
    transform = None

    for index, (angle, offset) in enumerate([(0.3, 1.0), (-0.2, -0.5), (0.4, 0.2)]):
        rotation = rotation_matrix(dimension, angle)
        if dimension == 3:
            # Use different axes so the 3D rotations do not commute either.
            rotation = np.roll(np.roll(rotation, index, axis=0), index, axis=1)
        translation = offset * np.arange(1, dimension + 1, dtype=float)
        previous_transform = transform
        previous_values = None if transform is None else transform.copy()
        transform = m.update_homogeneous_matrix(transform, rotation, translation)
        transformed_points = rotation @ transformed_points + translation[:, None]

        np.testing.assert_allclose(
            transform[:-1, :-1] @ points + transform[:-1, -1, None],
            transformed_points, atol=1e-12)
        if previous_transform is not None:
            np.testing.assert_array_equal(previous_transform, previous_values)


@pytest.mark.parametrize("dimension", [2, 3])
def test_single_icp_iteration_recovers_rigid_motion(dimension, monkeypatch):
    monkeypatch.setattr(m, "show_animation", False)
    monkeypatch.setattr(m, "MAX_ITER", 1)
    rng = np.random.default_rng(10)
    previous_points = rng.uniform(-5.0, 5.0, (dimension, 40))
    rotation = rotation_matrix(dimension, 0.001)
    translation = 0.001 * np.arange(1, dimension + 1)
    current_points = rotation @ previous_points + translation[:, None]

    estimated_rotation, estimated_translation = m.icp_matching(
        previous_points, current_points)

    np.testing.assert_allclose(estimated_rotation, rotation.T, atol=1e-12)
    np.testing.assert_allclose(
        estimated_translation, -rotation.T @ translation, atol=1e-12)


@pytest.mark.parametrize("dimension", [2, 3])
@pytest.mark.parametrize("seed", [1, 7, 42])
def test_icp_returned_transform_aligns_noiseless_cloud(dimension, seed, monkeypatch):
    monkeypatch.setattr(m, "show_animation", False)
    rng = np.random.default_rng(seed)
    previous_points = rng.uniform(-5.0, 5.0, (dimension, 40))
    rotation = rotation_matrix(dimension, 0.15)
    translation = 0.2 * np.arange(1, dimension + 1)
    current_points = rotation @ previous_points + translation[:, None]
    original_points = current_points.copy()

    estimated_rotation, estimated_translation = m.icp_matching(
        previous_points, current_points)

    np.testing.assert_allclose(estimated_rotation, rotation.T, atol=1e-10)
    np.testing.assert_allclose(
        estimated_translation, -rotation.T @ translation, atol=1e-10)
    np.testing.assert_allclose(
        estimated_rotation @ current_points + estimated_translation[:, None],
        previous_points, atol=1e-10)
    np.testing.assert_array_equal(current_points, original_points)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
