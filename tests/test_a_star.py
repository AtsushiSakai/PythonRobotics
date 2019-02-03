from unittest import TestCase

from PathPlanning.AStar import a_star as m


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
