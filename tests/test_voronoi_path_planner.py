from unittest import TestCase
from PathPlanning.VoronoiRoadMap import voronoi_road_map as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
