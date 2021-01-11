import conftest  # Add root path to sys.path
from unittest import TestCase
from Bipedal.bipedal_planner import bipedal_planner as m


def test_1():
    bipedal_planner = m.BipedalPlanner()

    footsteps = [[0.0, 0.2, 0.0],
                 [0.3, 0.2, 0.0],
                 [0.3, 0.2, 0.2],
                 [0.3, 0.2, 0.2],
                 [0.0, 0.2, 0.2]]
    bipedal_planner.set_ref_footsteps(footsteps)
    bipedal_planner.walk(plot=False)
