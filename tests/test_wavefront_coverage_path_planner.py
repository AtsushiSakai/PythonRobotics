import conftest  # Add root path to sys.path
import os
import matplotlib.pyplot as plt
from PathPlanning.WavefrontCPP import wavefront_coverage_path_planner

wavefront_coverage_path_planner.do_animation = False


def wavefront_cpp(img, start, goal):
    num_free = 0
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            num_free += 1 - img[i][j]

    DT = wavefront_coverage_path_planner.transform(
        img, goal, transform_type='distance')
    DT_path = wavefront_coverage_path_planner.wavefront(DT, start, goal)
    assert len(DT_path) == num_free  # assert complete coverage

    PT = wavefront_coverage_path_planner.transform(
        img, goal, transform_type='path', alpha=0.01)
    PT_path = wavefront_coverage_path_planner.wavefront(PT, start, goal)
    assert len(PT_path) == num_free  # assert complete coverage


def test_wavefront_CPP_1():
    img_dir = os.path.dirname(
        os.path.abspath(__file__)) + "/../PathPlanning/WavefrontCPP"
    img = plt.imread(os.path.join(img_dir, 'map', 'test.png'))
    img = 1 - img

    start = (43, 0)
    goal = (0, 0)

    wavefront_cpp(img, start, goal)


def test_wavefront_CPP_2():
    img_dir = os.path.dirname(
        os.path.abspath(__file__)) + "/../PathPlanning/WavefrontCPP"
    img = plt.imread(os.path.join(img_dir, 'map', 'test_2.png'))
    img = 1 - img

    start = (10, 0)
    goal = (10, 40)

    wavefront_cpp(img, start, goal)


def test_wavefront_CPP_3():
    img_dir = os.path.dirname(
        os.path.abspath(__file__)) + "/../PathPlanning/WavefrontCPP"
    img = plt.imread(os.path.join(img_dir, 'map', 'test_3.png'))
    img = 1 - img

    start = (0, 0)
    goal = (30, 30)

    wavefront_cpp(img, start, goal)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
