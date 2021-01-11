import conftest
from PathPlanning.AStar import a_star_searching_from_two_side as m


def test1():
    m.show_animation = False
    m.main(800)


def test2():
    m.show_animation = False
    m.main(5000)  # increase obstacle number, block path


if __name__ == '__main__':
    conftest.run_this_test(__file__)
