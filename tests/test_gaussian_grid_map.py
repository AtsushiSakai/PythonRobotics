from unittest import TestCase

from Mapping.gaussian_grid_map import gaussian_grid_map as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
