import conftest  # Add root path to sys.path
from PathPlanning.RRTStarReedsShepp import rrt_star_reeds_shepp as m


def test1():
    m.show_animation = False
    m.main(max_iter=5)


if __name__ == '__main__':
    conftest.run_this_test(__file__)
