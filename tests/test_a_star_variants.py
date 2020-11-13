import PathPlanning.AStar.a_star_variants as astar
from unittest import TestCase
import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../")


class Test(TestCase):

    def test(self):
        # A* with beam search
        astar.show_animation = False

        astar.use_beam_search = True
        astar.main()
        self.reset_all()

        # A* with iterative deepening
        astar.use_iterative_deepening = True
        astar.main()
        self.reset_all()

        # A* with dynamic weighting
        astar.use_dynamic_weighting = True
        astar.main()
        self.reset_all()

        # theta*
        astar.use_theta_star = True
        astar.main()
        self.reset_all()

        # A* with jump point
        astar.use_jump_point = True
        astar.main()
        self.reset_all()

    @staticmethod
    def reset_all():
        astar.show_animation = False
        astar.use_beam_search = False
        astar.use_iterative_deepening = False
        astar.use_dynamic_weighting = False
        astar.use_theta_star = False
        astar.use_jump_point = False


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test()
