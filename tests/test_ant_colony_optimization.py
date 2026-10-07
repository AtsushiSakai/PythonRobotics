import conftest
from PathPlanning.AntColonyOptimization import ant_colony_optimization as m


def test_1():
    m.show_animation = False
    m.N_ITERATIONS = 20  # Reduce for faster testing
    m.N_ANTS = 10
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
    