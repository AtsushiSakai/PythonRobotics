from unittest import TestCase

import sys
sys.path.append("./PathTracking/lqr/")

from PathTracking.lqr import lqr_tracking as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
