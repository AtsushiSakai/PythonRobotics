from unittest import TestCase
import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../")
sys.path.append(os.path.dirname(os.path.abspath(__file__))
                + "/../PathPlanning/HybridAStar")
try:
    from PathPlanning.HybridAStar import hybrid_a_star as m
except Exception:
    raise


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
