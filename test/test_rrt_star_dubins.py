import conftest  # Add root path to sys.path
from PathPlanning.RRTStarDubins import rrt_star_dubins as m


def test1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
