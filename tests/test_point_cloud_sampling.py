import numpy as np
import pytest

import conftest  # Add root path to sys.path
from Mapping.point_cloud_sampling import point_cloud_sampling as m


def test_1(capsys):
    m.do_plot = False
    m.main()
    captured = capsys.readouterr()
    assert "voxel_sampling_points.shape=(27, 3)" in captured.out
    assert "farthest_sampling_points.shape=(20, 3)" in captured.out
    assert "poisson_disk_points.shape=(20, 3)" in captured.out


@pytest.mark.parametrize("points", [np.empty((0, 3)), np.ones((4, 2), dtype=np.int32)])
def test_farthest_sampling_zero_count(points):
    sampled = m.farthest_point_sampling(points, 0, seed=0)
    assert sampled.shape == (0, points.shape[1])
    assert sampled.dtype == points.dtype


@pytest.mark.parametrize("count", [-1, 3])
def test_farthest_sampling_rejects_unavailable_count(count):
    with pytest.raises(ValueError, match="between 0 and"):
        m.farthest_point_sampling(np.zeros((2, 3)), count, seed=0)


def test_farthest_sampling_rejects_nonempty_sample_from_empty_cloud():
    with pytest.raises(ValueError, match="between 0 and"):
        m.farthest_point_sampling(np.empty((0, 3)), 1, seed=0)


@pytest.mark.parametrize("count", [1.5, 2.0, "2", None])
def test_farthest_sampling_requires_integer_count(count):
    with pytest.raises(TypeError):
        m.farthest_point_sampling(np.zeros((3, 2)), count, seed=0)


@pytest.mark.parametrize("seed, expected", [
    (0, [[16, 0], [0, 0], [9, 0], [4, 0]]),
    (42, [[0, 0], [16, 0], [9, 0], [4, 0]]),
])
def test_farthest_sampling_preserves_seeded_order(seed, expected):
    points = np.array([[0, 0], [1, 0], [4, 0], [9, 0], [16, 0]])
    np.testing.assert_array_equal(
        m.farthest_point_sampling(points, np.int64(4), seed), expected)


def test_farthest_sampling_duplicate_points_use_distinct_rows():
    points = np.array([[0, 0], [0, 0], [1, 0], [1, 0]])
    original = points.copy()
    sampled = m.farthest_point_sampling(points, len(points), seed=0)
    np.testing.assert_array_equal(np.sort(sampled, axis=0), points)
    np.testing.assert_array_equal(points, original)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
