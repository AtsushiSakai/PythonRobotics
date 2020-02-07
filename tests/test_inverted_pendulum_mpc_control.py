from unittest import TestCase

import sys
if 'cvxpy' in sys.modules:  # pragma: no cover
    sys.path.append("./InvertedPendulumCart/inverted_pendulum_mpc_control/")

    import inverted_pendulum_mpc_control as m

    print(__file__)

    class Test(TestCase):

        def test1(self):
            m.show_animation = False
            m.main()
