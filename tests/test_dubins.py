import conftest
from PathPlanning.DubinsPath import dubins_path_planning as m


def test_1():
    m.show_animation = True
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
