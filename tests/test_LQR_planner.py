
# Adding root path to sys.path
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from unittest import TestCase
from PathPlanning.LQRPlanner import LQRplanner as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.SHOW_ANIMATION = False
        m.main()
