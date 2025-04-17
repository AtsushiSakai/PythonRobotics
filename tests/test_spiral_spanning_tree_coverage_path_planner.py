import conftest  # Add root path to sys.path
import os
import matplotlib.pyplot as plt
from PathPlanning.SpiralSpanningTreeCPP \
    import spiral_spanning_tree_coverage_path_planner

spiral_spanning_tree_coverage_path_planner.do_animation = True


def spiral_stc_cpp(img, start):
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
    assert len(covered_nodes) == num_free / 4


def test_spiral_stc_cpp_1():
    img_dir = os.path.dirname(
        os.path.abspath(__file__)) + \
        "/../PathPlanning/SpiralSpanningTreeCPP"
    img = plt.imread(os.path.join(img_dir, 'map', 'test.png'))
    start = (0, 0)
    spiral_stc_cpp(img, start)


def test_spiral_stc_cpp_2():
    img_dir = os.path.dirname(
        os.path.abspath(__file__)) + \
        "/../PathPlanning/SpiralSpanningTreeCPP"
    img = plt.imread(os.path.join(img_dir, 'map', 'test_2.png'))
    start = (10, 0)
    spiral_stc_cpp(img, start)


def test_spiral_stc_cpp_3():
    img_dir = os.path.dirname(
        os.path.abspath(__file__)) + \
        "/../PathPlanning/SpiralSpanningTreeCPP"
    img = plt.imread(os.path.join(img_dir, 'map', 'test_3.png'))
    start = (0, 0)
    spiral_stc_cpp(img, start)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
