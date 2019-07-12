from unittest import TestCase
import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../")
try:
    from PathPlanning.BatchInformedRRTStar import batch_informed_rrtstar as m
except:
    raise

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main(maxIter=10)


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
