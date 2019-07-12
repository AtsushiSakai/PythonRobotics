from unittest import TestCase

import sys

if 'cvxpy' in sys.modules:  # pragma: no cover
    sys.path.append("./AerialNavigation/rocket_powered_landing/")

    from AerialNavigation.rocket_powered_landing import rocket_powered_landing as m
    print(__file__)

    class Test(TestCase):

        def test1(self):
            m.show_animation = False
            m.main()
