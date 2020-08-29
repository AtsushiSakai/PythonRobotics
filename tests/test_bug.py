from unittest import TestCase
import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../")
try:
    from PathPlanning.Bug_Planning import bug as b
except:
    raise


class Test(TestCase):

    def test(self):
        b.show_animation = False
        b.main(bug0=True, bug1=True, bug2=True)


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test()
