import conftest  # Add root path to sys.path
import numpy as np
import pytest
from PathPlanning.QuinticPolynomialsPlanner import quintic_polynomials_planner as m


def test1():
    m.show_animation = False
    m.main()


@pytest.mark.parametrize("dt", [0.5, 0.7, 6.0])
def test_quintic_trajectory_includes_exact_terminal_time(dt):
    m.show_animation = False
    time, rx, ry, ryaw, rv, ra, rj = m.quintic_polynomials_planner(
        sx=0.0, sy=0.0, syaw=0.0, sv=0.0, sa=0.0,
        gx=10.0, gy=5.0, gyaw=0.0, gv=0.0, ga=0.0,
        max_accel=1000.0, max_jerk=1000.0, dt=dt
    )

    assert time[-1] == m.MIN_T
    assert np.all(np.diff(time) > 0)
    assert np.all(np.asarray(time) <= m.MIN_T)
    assert len({len(values) for values in (time, rx, ry, ryaw, rv, ra, rj)}) == 1
    np.testing.assert_allclose([rx[-1], ry[-1]], [10.0, 5.0], atol=1e-9)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
