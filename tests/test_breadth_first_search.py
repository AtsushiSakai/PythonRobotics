import conftest  # Add root path to sys.path
from unittest import TestCase
from PathPlanning.BreadthFirstSearch import breadth_first_search as m


def test_1():
    m.show_animation = False
    m.main()
