import conftest  # noqa
import numpy as np
import pytest
from Mapping.DistanceMap import distance_map as m


def test_compute_sdf():
    """Test the computation of Signed Distance Field (SDF)"""
    # Create a simple obstacle map for testing
    obstacles = np.array([[0, 0, 0], [0, 1, 0], [0, 0, 0]])

    sdf = m.compute_sdf(obstacles)

    # Verify basic properties of SDF
    assert sdf.shape == obstacles.shape, "SDF should have the same shape as input map"
    assert np.all(np.isfinite(sdf)), "SDF should not contain infinite values"

    # Verify SDF value is negative at obstacle position
    assert sdf[1, 1] < 0, "SDF value should be negative at obstacle position"

    # Verify SDF value is positive in free space
    assert sdf[0, 0] > 0, "SDF value should be positive in free space"


def test_compute_udf():
    """Test the computation of Unsigned Distance Field (UDF)"""
    # Create obstacle map for testing
    obstacles = np.array([[0, 0, 0], [0, 1, 0], [0, 0, 0]])

    udf = m.compute_udf(obstacles)

    # Verify basic properties of UDF
    assert udf.shape == obstacles.shape, "UDF should have the same shape as input map"
    assert np.all(np.isfinite(udf)), "UDF should not contain infinite values"
    assert np.all(udf >= 0), "All UDF values should be non-negative"

    # Verify UDF value is 0 at obstacle position
    assert np.abs(udf[1, 1]) < 1e-10, "UDF value should be 0 at obstacle position"

    # Verify UDF value is 1 for adjacent cells
    assert np.abs(udf[0, 1] - 1.0) < 1e-10, (
        "UDF value should be 1 for cells adjacent to obstacle"
    )
    assert np.abs(udf[1, 0] - 1.0) < 1e-10, (
        "UDF value should be 1 for cells adjacent to obstacle"
    )
    assert np.abs(udf[1, 2] - 1.0) < 1e-10, (
        "UDF value should be 1 for cells adjacent to obstacle"
    )
    assert np.abs(udf[2, 1] - 1.0) < 1e-10, (
        "UDF value should be 1 for cells adjacent to obstacle"
    )


def test_dt():
    """Test the computation of 1D distance transform"""
    # Create test data
    d = np.array([m.INF, 0, m.INF])
    m.dt(d)

    # Verify distance transform results
    assert np.all(np.isfinite(d)), (
        "Distance transform result should not contain infinite values"
    )
    assert d[1] == 0, "Distance at obstacle position should be 0"
    assert d[0] == 1, "Distance at adjacent position should be 1"
    assert d[2] == 1, "Distance at adjacent position should be 1"


def test_compute_sdf_empty():
    """Test SDF computation with empty map"""
    # Test with empty map (no obstacles)
    empty_map = np.zeros((5, 5))
    sdf = m.compute_sdf(empty_map)

    assert np.all(sdf > 0), "All SDF values should be positive for empty map"
    assert sdf.shape == empty_map.shape, "Output shape should match input shape"


def test_compute_sdf_full():
    """Test SDF computation with fully occupied map"""
    # Test with fully occupied map
    full_map = np.ones((5, 5))
    sdf = m.compute_sdf(full_map)

    assert np.all(sdf < 0), "All SDF values should be negative for fully occupied map"
    assert sdf.shape == full_map.shape, "Output shape should match input shape"


def test_compute_udf_invalid_input():
    """Test UDF computation with invalid input values"""
    # Test with invalid values (not 0 or 1)
    invalid_map = np.array([[0, 2, 0], [0, -1, 0], [0, 0.5, 0]])

    try:
        m.compute_udf(invalid_map)
        assert False, "Should raise ValueError for invalid input values"
    except ValueError:
        pass


def test_compute_udf_empty():
    """Test UDF computation with empty map"""
    # Test with empty map
    empty_map = np.zeros((5, 5))
    udf = m.compute_udf(empty_map)

    assert np.all(udf > 0), "All UDF values should be positive for empty map"
    assert np.all(np.isfinite(udf)), "UDF should not contain infinite values"


def test_main():
    """Test the execution of main function"""
    m.ENABLE_PLOT = False
    m.main()


def test_dt_preserves_parabola_costs():
    distances = np.array([0.0, 4.0, 9.0, 9.0])
    assert m.dt(distances) is None
    np.testing.assert_array_equal(distances, [0.0, 1.0, 4.0, 8.0])


def test_dt_matches_brute_force():
    rng = np.random.default_rng(0)
    for size in range(1, 17):
        for _ in range(10):
            costs = rng.uniform(0.0, 30.0, size)
            expected = [min((q - j) ** 2 + costs[j] for j in range(size))
                        for q in range(size)]
            distances = costs.copy()
            m.dt(distances)
            np.testing.assert_allclose(distances, expected)


def test_dt_updates_strided_view():
    values = np.array([[0.0, 10.0], [4.0, 20.0],
                       [9.0, 30.0], [9.0, 40.0]])
    m.dt(values[:, 0])
    np.testing.assert_array_equal(values[:, 0], [0.0, 1.0, 4.0, 8.0])
    np.testing.assert_array_equal(values[:, 1], [10.0, 20.0, 30.0, 40.0])


def brute_force_udf(obstacles):
    obstacle_indices = np.argwhere(obstacles)
    return np.array([
        min(np.sum((np.array(index) - obstacle) ** 2)
            for obstacle in obstacle_indices) ** 0.5
        for index in np.ndindex(obstacles.shape)
    ]).reshape(obstacles.shape)


def test_distance_fields_preserve_column_costs():
    obstacles = np.array([[1, 0, 0, 0], [0, 0, 1, 0],
                          [0, 0, 0, 1], [0, 0, 0, 1]])
    expected = brute_force_udf(obstacles)
    assert expected[3, 0] == np.sqrt(8)
    np.testing.assert_allclose(m.compute_udf(obstacles), expected)
    np.testing.assert_allclose(m.compute_sdf(obstacles),
                               expected - brute_force_udf(obstacles == 0))


@pytest.mark.parametrize("shape", [(1, 7), (7, 1), (3, 5), (5, 3), (7, 8)])
def test_distance_fields_match_brute_force(shape):
    rng = np.random.default_rng(0)
    for _ in range(10):
        obstacles = rng.integers(0, 2, size=shape).astype(bool)
        obstacles.flat[0] = True
        obstacles.flat[-1] = False
        original = obstacles.copy()
        expected_udf = brute_force_udf(obstacles)
        expected_sdf = expected_udf - brute_force_udf(~obstacles)
        np.testing.assert_allclose(m.compute_udf(obstacles), expected_udf)
        np.testing.assert_allclose(m.compute_sdf(obstacles), expected_sdf)
        np.testing.assert_allclose(m.compute_udf_scipy(obstacles), expected_udf)
        np.testing.assert_allclose(m.compute_sdf_scipy(obstacles), expected_sdf)
        np.testing.assert_array_equal(obstacles, original)


@pytest.mark.parametrize("occupied", [False, True])
@pytest.mark.parametrize("shape", [(1, 1), (1, 5), (5, 1), (3, 4)])
def test_uniform_distance_fields(shape, occupied):
    obstacles = np.full(shape, occupied)
    expected_udf = 0.0 if occupied else np.sqrt(m.INF)
    expected_sdf = -np.sqrt(m.INF) if occupied else np.sqrt(m.INF)
    np.testing.assert_array_equal(m.compute_udf(obstacles),
                                  np.full(shape, expected_udf))
    np.testing.assert_array_equal(m.compute_sdf(obstacles),
                                  np.full(shape, expected_sdf))


if __name__ == "__main__":
    conftest.run_this_test(__file__)
