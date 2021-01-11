import conftest  # Add root path to sys.path
from unittest import TestCase
from PathPlanning.Dijkstra import dijkstra as m


def test_1():
    m.show_animation = False
    m.main()
