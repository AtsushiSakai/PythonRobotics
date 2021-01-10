import conftest  # Add root path to sys.path
from unittest import TestCase
from PathTracking.cgmres_nmpc import cgmres_nmpc as m


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
