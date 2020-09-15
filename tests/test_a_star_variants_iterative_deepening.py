import a_star_variants as astar
from unittest import TestCase
import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../")


class Test(TestCase):

    def test(self):
        # A* with dynamic weighting
        astar.show_animation = False
        astar.use_iterative_deepening = True
        astar.main()


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test()
