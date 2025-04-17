import conftest  # noqa
import numpy as np
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


if __name__ == "__main__":
    conftest.run_this_test(__file__)
