from unittest import TestCase

from PathPlanning.PotentialFieldPlanning import potential_field_planning as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
