import conftest  # Add root path to sys.path
from PathPlanning.StateLatticePlanner import state_lattice_planner as m
from PathPlanning.ModelPredictiveTrajectoryGenerator \
    import model_predictive_trajectory_generator as m2


def test1():
    m.show_animation = False
    m2.show_animation = False
    m.main()
