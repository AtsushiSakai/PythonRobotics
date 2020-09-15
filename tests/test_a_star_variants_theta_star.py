from PathPlanning.AStar import a_star_variants as astar
from unittest import TestCase
import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../")


class Test(TestCase):

    def test(self):
        # Theta*
        astar.show_animation = False
        astar.use_theta_star = True
        astar.main()


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test()
