from unittest import TestCase

from Mapping.circle_fitting import circle_fitting as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
