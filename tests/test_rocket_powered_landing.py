import conftest  # Add root path to sys.path
import warnings

import numpy as np

from AerialNavigation.rocket_powered_landing import rocket_powered_landing as m


def test1():
    m.show_animation = False
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore",
            message="You are solving a parameterized problem that is not DPP",
            category=UserWarning,
        )
        warnings.filterwarnings(
            "ignore",
            message="Solution may be inaccurate",
            category=UserWarning,
        )
        m.main(rng=np.random.default_rng(1234))


if __name__ == '__main__':
    conftest.run_this_test(__file__)
