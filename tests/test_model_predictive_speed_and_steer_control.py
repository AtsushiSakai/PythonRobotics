from unittest import TestCase

import sys
sys.path.append("./PathTracking/model_predictive_speed_and_steer_control/")

from PathTracking.model_predictive_speed_and_steer_control import model_predictive_speed_and_steer_control as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
        m.main2()
