import conftest  # Add root path to sys.path
from unittest import TestCase
from PathPlanning.BugPlanning import bug as m


def test_1():
    m.show_animation = False
    m.main(bug_0=True, bug_1=True, bug_2=True)
