# Adding root path to sys.path
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from unittest import TestCase
from PathPlanning.AStar import a_star_searching_from_two_side as m


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main(800)

    def test2(self):
        m.show_animation = False
        m.main(5000)  # increase obstacle number, block path


if __name__ == '__main__':
    test = Test()
    test.test1()
    test.test2()
