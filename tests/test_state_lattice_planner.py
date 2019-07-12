from unittest import TestCase

import sys

import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../PathPlanning/ModelPredictiveTrajectoryGenerator/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../PathPlanning/StateLatticePlanner/")

try:
    import state_lattice_planner as m
    import model_predictive_trajectory_generator as m2
except:
    raise

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m2.show_animation = False
        m.main()


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
