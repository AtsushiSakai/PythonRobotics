import conftest  # Add root path to sys.path
from unittest import TestCase
from PathPlanning.LQRPlanner import LQRplanner as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.SHOW_ANIMATION = False
        m.main()
