import os
import sys
import matplotlib.pyplot as plt
from unittest import TestCase

sys.path.append(os.path.dirname(
    os.path.abspath(__file__)) + "/../PathPlanning/SpiralSpanningTreeCPP")
try:
    import spiral_spanning_tree_coverage_path_planner
except ImportError:
    raise

spiral_spanning_tree_coverage_path_planner.do_animation = True


class TestPlanning(TestCase):
    def spiral_stc_cpp(self, img, start):
        num_free = 0
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                num_free += img[i][j]

        STC_planner = spiral_spanning_tree_coverage_path_planner.\
            SpiralSpanningTreeCoveragePlanner(img)

        edge, route, path = STC_planner.plan(start)

        covered_nodes = set()
        for p, q in edge:
            covered_nodes.add(p)
            covered_nodes.add(q)

        # assert complete coverage
        self.assertEqual(len(covered_nodes), num_free / 4)

    def test_spiral_stc_cpp_1(self):
        img_dir = os.path.dirname(
            os.path.abspath(__file__)) + \
            "/../PathPlanning/SpiralSpanningTreeCPP"
        img = plt.imread(os.path.join(img_dir, 'map', 'test.png'))
        start = (0, 0)
        self.spiral_stc_cpp(img, start)

    def test_spiral_stc_cpp_2(self):
        img_dir = os.path.dirname(
            os.path.abspath(__file__)) + \
            "/../PathPlanning/SpiralSpanningTreeCPP"
        img = plt.imread(os.path.join(img_dir, 'map', 'test_2.png'))
        start = (10, 0)
        self.spiral_stc_cpp(img, start)

    def test_spiral_stc_cpp_3(self):
        img_dir = os.path.dirname(
            os.path.abspath(__file__)) + \
            "/../PathPlanning/SpiralSpanningTreeCPP"
        img = plt.imread(os.path.join(img_dir, 'map', 'test_3.png'))
        start = (0, 0)
        self.spiral_stc_cpp(img, start)
