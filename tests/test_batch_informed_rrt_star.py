import conftest  # Add root path to sys.path
from unittest import TestCase
from PathPlanning.BatchInformedRRTStar import batch_informed_rrtstar as m


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        random.seed(12345)
        m.main(maxIter=10)


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
