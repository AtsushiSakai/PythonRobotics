from unittest import TestCase

import sys
sys.path.append("./PathPlanning/BezierPath/")

from PathPlanning.BezierPath import bezier_path as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
        m.main2()
