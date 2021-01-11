import conftest  # Add root path to sys.path
from unittest import TestCase
from PathPlanning.AStar import a_star as m


def test_1():
    m.show_animation = False
    m.main()
