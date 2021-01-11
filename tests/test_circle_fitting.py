import conftest  # Add root path to sys.path
from unittest import TestCase
from Mapping.circle_fitting import circle_fitting as m


def test_1():
    m.show_animation = False
    m.main()
