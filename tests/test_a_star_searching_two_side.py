import conftest  # Add root path to sys.path
from unittest import TestCase
from PathPlanning.AStar import a_star_searching_from_two_side as m


def test1():
    m.show_animation = False
    m.main(800)


def test2():
    m.show_animation = False
    m.main(5000)  # increase obstacle number, block path
