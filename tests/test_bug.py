import conftest
from PathPlanning.BugPlanning import bug as m


def test_1():
    m.show_animation = False
    m.main(bug_0=True, bug_1=True, bug_2=True)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
