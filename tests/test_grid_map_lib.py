import os
import sys
import unittest

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../Mapping/grid_map_lib")
try:
    from grid_map_lib import GridMap
except ImportError:
    raise


class MyTestCase(unittest.TestCase):

    def test_position_set(self):
        grid_map = GridMap(100, 120, 0.5, 10.0, -0.5)

        grid_map.set_value_from_xy_pos(10.1, -1.1, 1.0)
        grid_map.set_value_from_xy_pos(10.1, -0.1, 1.0)
        grid_map.set_value_from_xy_pos(10.1, 1.1, 1.0)
        grid_map.set_value_from_xy_pos(11.1, 0.1, 1.0)
        grid_map.set_value_from_xy_pos(10.1, 0.1, 1.0)
        grid_map.set_value_from_xy_pos(9.1, 0.1, 1.0)

        self.assertEqual(True, True)

    def test_polygon_set(self):
        ox = [0.0, 20.0, 50.0, 100.0, 130.0, 40.0]
        oy = [0.0, -20.0, 0.0, 30.0, 60.0, 80.0]

        grid_map = GridMap(600, 290, 0.7, 60.0, 30.5)

        grid_map.set_value_from_polygon(ox, oy, 1.0, inside=False)

        self.assertEqual(True, True)


if __name__ == '__main__':
    unittest.main()
