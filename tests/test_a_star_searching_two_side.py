from unittest import TestCase
import os
import sys

sys.path.append(os.path.dirname(__file__) + '/../')

try:
    from PathPlanning.AStar import A_Star_searching_from_two_side as m
except ImportError:
    raise


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main([60, 60], [0, 0], 1500)


if __name__ == '__main__':
    test = Test()
    test.test1()
