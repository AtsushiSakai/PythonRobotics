import conftest  # Add root path to sys.path
from PathPlanning.DepthFirstSearch import depth_first_search as m


def test_1():
    m.show_animation = False
    m.main()
