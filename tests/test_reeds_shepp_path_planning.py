from unittest import TestCase
from PathPlanning.ReedsSheppPath import reeds_shepp_path_planning as m


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
