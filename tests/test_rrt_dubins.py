import conftest  # Add root path to sys.path
from PathPlanning.RRTDubins import rrt_dubins as m


def test1():
    m.show_animation = False
    m.main()


def test_start_already_satisfies_goal():
    planner = m.RRTDubins(
        start=[0.0, 0.0, 0.0],
        goal=[0.0, 0.0, 0.0],
        obstacle_list=[],
        rand_area=[-1.0, 1.0],
        max_iter=0,
    )

    path = planner.planning(animation=False)

    assert path is not None
    assert path[0] == [0.0, 0.0]
    assert path[-1] == [0.0, 0.0]


if __name__ == '__main__':
    conftest.run_this_test(__file__)
