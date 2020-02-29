import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__))
                + "/../PathPlanning/VisibilityRoadMap/")

from unittest import TestCase
from PathPlanning.VisibilityRoadMap import visibility_road_map as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
