from unittest import TestCase

from Localization.particle_filter import particle_filter as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
