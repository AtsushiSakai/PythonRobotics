from unittest import TestCase

import sys
if 'cvxpy' in sys.modules:  # pragma: no cover
    sys.path.append("./PathTracking/cgmres_nmpc/")

    from PathTracking.cgmres_nmpc import cgmres_nmpc as m

    print(__file__)

    class Test(TestCase):

        def test1(self):
            m.show_animation = False
            m.main()
