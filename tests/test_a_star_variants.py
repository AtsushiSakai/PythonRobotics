import PathPlanning.AStar.a_star_variants as astar
from unittest import TestCase
import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../")


class Test(TestCase):

    def test(self):
        astar.show_animation = False
        astar.use_jump_point = True
        astar.main()


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test()
