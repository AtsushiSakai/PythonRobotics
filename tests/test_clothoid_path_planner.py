import unittest
import numpy as np
from math import pi
from PathPlanning.ClothoidPath.clothoid_path_planner import Point, generate_clothoid_paths, draw_clothoids

class TestClothoidPathPlanner(unittest.TestCase):

    def setUp(self):
        self.start_point = Point(0, 0)
        self.goal_point = Point(10, 0)
        self.num_path_points = 100
        self.start_orientation_list = [0.0]
        self.goal_orientation_list = np.linspace(-pi, pi, 75)
        self.clothoidal_paths = generate_clothoid_paths(
            self.start_point, self.start_orientation_list,
            self.goal_point, self.goal_orientation_list,
            self.num_path_points)

    def test_draw_clothoids_no_save(self):
        draw_clothoids(self.start_point, self.goal_point, self.num_path_points, self.clothoidal_paths, save_animation=False)

    def test_draw_clothoids_save(self):
        draw_clothoids(self.start_point, self.goal_point, self.num_path_points, self.clothoidal_paths, save_animation=True)

if __name__ == '__main__':
    unittest.main()
