import conftest  # Add root path to sys.path
import numpy as np
from PathPlanning.PotentialFieldPlanning import potential_field_planning as m


def test1():
    m.show_animation = False
    m.main()


def test_obstacle_free_potential_field():
    # A free map has only attractive potential; there is no nearest obstacle.
    gx, gy, sx, sy = 5.0, 5.0, 0.0, 0.0
    assert m.calc_repulsive_potential(1.0, 2.0, [], [], 1.0) == 0.0

    field, minx, miny = m.calc_potential_field(
        gx, gy, [], [], 1.0, 1.0, sx, sy
    )
    assert np.isfinite(field).all()
    np.testing.assert_allclose(
        field[0][0], m.calc_attractive_potential(minx, miny, gx, gy)
    )


def test_obstacle_free_planning():
    m.show_animation = False
    sx, sy, gx, gy = 0.0, 0.0, 5.0, 5.0
    rx, ry = m.potential_field_planning(sx, sy, gx, gy, [], [], 1.0, 1.0)

    np.testing.assert_allclose([rx[0], ry[0]], [sx, sy])
    assert np.hypot(rx[-1] - gx, ry[-1] - gy) < 1.0


if __name__ == '__main__':
    conftest.run_this_test(__file__)
