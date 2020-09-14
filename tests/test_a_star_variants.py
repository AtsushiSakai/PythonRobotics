import PathPlanning.AStar.a_star_variants as astar
from unittest import TestCase
import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../")


class Test(TestCase):

    def test(self):
        # normal A*
        astar.show_animation = True
        astar.main()

        # A* with beam search
        astar.use_beam_search = True
        astar.main()

        # A* with iterative deepening
        self.reset_all()
        astar.use_iterative_deepening = True
        astar.main()

        # A* with dynamic weighting
        self.reset_all()
        astar.use_dynamic_weighting = True
        astar.main()

        # Theta*
        self.reset_all()
        astar.use_theta_star = True
        astar.main()

        # A* with jump point
        self.reset_all()
        astar.use_jump_point = True
        astar.main()

    def reset_all(self):
        astar.use_beam_search = False
        astar.use_iterative_deepening = False
        astar.use_dynamic_weighting = False
        astar.use_theta_star = False
        astar.use_jump_point = False


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test()
