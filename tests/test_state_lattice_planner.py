import sys
from unittest import TestCase

sys.path.append("./PathPlanning/ModelPredictiveTrajectoryGenerator/")
sys.path.append("./PathPlanning/StateLatticePlanner/")

from PathPlanning.StateLatticePlanner import state_lattice_planner as m


m.table_path = "./PathPlanning/StateLatticePlanner/lookuptable.csv"

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
