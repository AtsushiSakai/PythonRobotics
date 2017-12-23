from unittest import TestCase
from PathPlanning.ProbabilisticRoadMap import probabilistic_road_map


class Test(TestCase):

    def test1(self):
        probabilistic_road_map.show_animation = False
        probabilistic_road_map.main()
