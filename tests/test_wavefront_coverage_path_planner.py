import os
import sys
import matplotlib.pyplot as plt
from unittest import TestCase

sys.path.append(os.path.dirname(
    os.path.abspath(__file__)) + "/../PathPlanning/WavefrontCPP")
try:
    import wavefront_coverage_path_planner
except ImportError:
    raise

wavefront_coverage_path_planner.do_animation = False


class TestPlanning(TestCase):
    def wavefront_cpp(self, img, start, goal):
        num_free = 0
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                num_free += 1 - img[i][j]

        DT = wavefront_coverage_path_planner.transform(
            img, goal, transform_type='distance')
        DT_path = wavefront_coverage_path_planner.wavefront(DT, start, goal)
        self.assertEqual(len(DT_path), num_free)  # assert complete coverage

        PT = wavefront_coverage_path_planner.transform(
            img, goal, transform_type='path', alpha=0.01)
        PT_path = wavefront_coverage_path_planner.wavefront(PT, start, goal)
        self.assertEqual(len(PT_path), num_free)  # assert complete coverage

    def test_wavefront_CPP_1(self):
        img_dir = os.path.dirname(
            os.path.abspath(__file__)) + "/../PathPlanning/WavefrontCPP"
        img = plt.imread(os.path.join(img_dir, 'map', 'test.png'))
        img = 1 - img

        start = (43, 0)
        goal = (0, 0)

        self.wavefront_cpp(img, start, goal)

    def test_wavefront_CPP_2(self):
        img_dir = os.path.dirname(
            os.path.abspath(__file__)) + "/../PathPlanning/WavefrontCPP"
        img = plt.imread(os.path.join(img_dir, 'map', 'test_2.png'))
        img = 1 - img

        start = (10, 0)
        goal = (10, 40)

        self.wavefront_cpp(img, start, goal)

    def test_wavefront_CPP_3(self):
        img_dir = os.path.dirname(
            os.path.abspath(__file__)) + "/../PathPlanning/WavefrontCPP"
        img = plt.imread(os.path.join(img_dir, 'map', 'test_3.png'))
        img = 1 - img

        start = (0, 0)
        goal = (30, 30)

        self.wavefront_cpp(img, start, goal)
