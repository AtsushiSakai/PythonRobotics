import pytest
import conftest
from Mapping.gaussian_grid_map import gaussian_grid_map as m


def test1():
    m.show_animation = False
    m.main()


def test_gaussian_tail_remains_positive():
    grid, minx, _, miny, _ = m.generate_gaussian_grid_map([0], [0], 1, 0.5)
    assert grid[-minx][-miny] == 0.5
    # At ten standard deviations, subtracting the CDF from one rounds to zero.
    assert grid[0][-miny] == pytest.approx(7.61985302416047e-24, abs=0)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
