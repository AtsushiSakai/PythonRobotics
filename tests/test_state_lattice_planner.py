import conftest  # Add root path to sys.path
import state_lattice_planner as m
import model_predictive_trajectory_generator as m2


def test1():
    m.show_animation = False
    m2.show_animation = False
    m.main()
