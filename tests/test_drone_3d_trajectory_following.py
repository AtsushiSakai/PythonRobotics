import conftest  # Add root path to sys.path
from unittest import TestCase
from AerialNavigation.drone_3d_trajectory_following import drone_3d_trajectory_following as m


def test1():
    m.show_animation = False
    m.main()
