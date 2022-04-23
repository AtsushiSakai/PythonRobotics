import conftest  # Add root path to sys.path
from PathPlanning.RRTStar import rrt_star as m


def test1():
    m.show_animation = False
    m.main()


def test_no_obstacle():
    obstacle_list = []

    # Set Initial parameters
    rrt_star = m.RRTStar(start=[0, 0],
                         goal=[6, 10],
                         rand_area=[-2, 15],
                         obstacle_list=obstacle_list)
    path = rrt_star.planning(animation=False)
    assert path is not None


def test_no_obstacle_and_robot_radius():
    obstacle_list = []

    # Set Initial parameters
    rrt_star = m.RRTStar(start=[0, 0],
                         goal=[6, 10],
                         rand_area=[-2, 15],
                         obstacle_list=obstacle_list,
                         robot_radius=0.8)
    path = rrt_star.planning(animation=False)
    assert path is not None


if __name__ == '__main__':
    conftest.run_this_test(__file__)
