from unittest import TestCase

from Mapping.raycasting_grid_map import raycasting_grid_map as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
