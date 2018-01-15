from unittest import TestCase

import sys
sys.path.append("./PathTracking/lqr_steer_control/")

from PathTracking.lqr_steer_control import lqr_steer_control as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
