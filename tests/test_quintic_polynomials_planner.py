from unittest import TestCase

from PathPlanning.QuinticPolynomialsPlanner import quintic_polynomials_planner as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
