from unittest import TestCase

from PathPlanning.BatchInformedRRTStar import batch_informed_rrtstar as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
