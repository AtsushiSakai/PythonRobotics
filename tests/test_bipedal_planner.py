from unittest import TestCase

import sys
sys.path.append("./Bipedal/bipedal_planner/")
try:
    from Bipedal.bipedal_planner import bipedal_planner as m
except Exception:
    raise

print(__file__)


class Test(TestCase):

    def test(self):
        bipedal_planner = m.BipedalPlanner()

        footsteps = [[0.0, 0.2, 0.0],
                     [0.3, 0.2, 0.0],
                     [0.3, 0.2, 0.2],
                     [0.3, 0.2, 0.2],
                     [0.0, 0.2, 0.2]]
        bipedal_planner.set_ref_footsteps(footsteps)
        bipedal_planner.walk(plot=False)
