import conftest  # Add root path to sys.path
from unittest import TestCase
from PathPlanning.BugPlanning import bug as m


class Test(TestCase):

    def test(self):
        b.show_animation = False
        b.main(bug_0=True, bug_1=True, bug_2=True)


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test()
