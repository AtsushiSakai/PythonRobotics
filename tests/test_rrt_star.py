from unittest import TestCase

from PathPlanning.RRTstar import rrt_star as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
