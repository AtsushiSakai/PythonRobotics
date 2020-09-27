import PathPlanning.FlowField.flowfield as flowfield
from unittest import TestCase
import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../")


class Test(TestCase):

    def test(self):
        flowfield.show_animation = False
        flowfield.main()


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test()
