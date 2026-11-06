import pytest
import conftest
from Mapping.circle_fitting import circle_fitting as m


def test_1():
    m.show_animation = False
    m.main()


@pytest.mark.parametrize("x, y", [([0, 2, 0, 1], [0, 0, 2]),
                                   ([0, 0, 2], [0, 2, 0, 1])])
def test_requires_paired_coordinates(x, y):
    with pytest.raises(ValueError, match="same number"):
        m.circle_fitting(x, y)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
