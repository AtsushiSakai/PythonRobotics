from unittest import TestCase

from PathPlanning.DynamicWindowApproach import dynamic_window_approach as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
