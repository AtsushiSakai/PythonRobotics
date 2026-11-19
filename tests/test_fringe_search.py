import conftest  # Add root path
from PathPlanning.FringeSearch import fringe_search


def test_1():
    fringe_search.show_animation = False
    fringe_search.main()


if __name__ == "__main__":
    conftest.run_this_test(__file__)
