from unittest import TestCase
import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../")
try:
    from PathPlanning.SpanningTree import st as m
except:
    raise


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
