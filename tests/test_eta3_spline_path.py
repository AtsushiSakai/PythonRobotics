
from unittest import TestCase
from PathPlanning.Eta3SplinePath import eta3_spline_path as m


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
