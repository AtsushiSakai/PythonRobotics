import conftest  # Add root path to sys.path
from unittest import TestCase
from PathPlanning.BreadthFirstSearch import breadth_first_search as m


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()


if __name__ == '__main__':  # pragma: no cover
    test = Test()
    test.test1()
