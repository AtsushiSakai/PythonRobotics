from unittest import TestCase

from PathPlanning.RRT import simple_rrt as m
from PathPlanning.RRT import rrt_with_pathsmoothing as m1

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()

    def test2(self):
        m1.show_animation = False
        m1.main()
