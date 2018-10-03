from unittest import TestCase

import sys
sys.path.append("./PathPlanning/LQRPlanner")

from PathPlanning.LQRPlanner import LQRplanner as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
