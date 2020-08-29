from PathPlanning.BugPlanning import bug as b
from unittest import TestCase
import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../")


class Test(TestCase):

    def test(self):
        b.show_animation = False
        b.main(bug_0=True, bug_1=True, bug_2=True)


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test()
