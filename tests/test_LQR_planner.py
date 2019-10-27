import sys
from unittest import TestCase

sys.path.append("./PathPlanning/LQRPlanner")

from PathPlanning.LQRPlanner import LQRplanner as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.SHOW_ANIMATION = False
        m.main()
