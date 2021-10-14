import conftest
from PathPlanning.BezierPath import bezier_path as m


def test_1():
    m.show_animation = False
    m.main()


def test_2():
    m.show_animation = False
    m.main2()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
