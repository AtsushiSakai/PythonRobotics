import conftest  # Add root path to sys.path
from unittest import TestCase
from PathPlanning.BezierPath import bezier_path as m


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
        m.main2()
