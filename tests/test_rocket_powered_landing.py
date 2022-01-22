import conftest  # Add root path to sys.path
import numpy as np
from numpy.testing import suppress_warnings

from AerialNavigation.rocket_powered_landing import rocket_powered_landing as m


def test1():
    m.show_animation = False
    with suppress_warnings() as sup:
        sup.filter(UserWarning,
                   "You are solving a parameterized problem that is not DPP"
                   )
        sup.filter(UserWarning,
                   "Solution may be inaccurate")
        m.main(rng=np.random.default_rng(1234))


if __name__ == '__main__':
    conftest.run_this_test(__file__)
