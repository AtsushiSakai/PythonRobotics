import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__))
                + "/../PathPlanning/VoronoiRoadMap/")

from unittest import TestCase
from PathPlanning.VoronoiRoadMap import voronoi_road_map as m


print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
