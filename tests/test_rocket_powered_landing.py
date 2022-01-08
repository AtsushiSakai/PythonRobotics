import conftest  # Add root path to sys.path
import sys
from numpy.testing import suppress_warnings

from AerialNavigation.rocket_powered_landing import rocket_powered_landing as m


def test1():
    m.show_animation = False
    with suppress_warnings() as sup:
        sup.filter(UserWarning,
                   "You are solving a parameterized problem that is not DPP"
                   )
        m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
