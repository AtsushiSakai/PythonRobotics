import conftest
from PathPlanning.HybridAStar import hybrid_a_star as m


def test1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
