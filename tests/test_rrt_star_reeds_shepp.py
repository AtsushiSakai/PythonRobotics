import conftest  # Add root path to sys.path
from PathPlanning.RRTStarReedsShepp import rrt_star_reeds_shepp as m


def test1():
    m.show_animation = False
    m.main(max_iter=5)

obstacleList = [
    (5, 5, 1),
    (4, 6, 1),
    (4, 8, 1),
    (4, 10, 1),
    (6, 5, 1),
    (7, 5, 1),
    (8, 6, 1),
    (8, 8, 1),
    (8, 10, 1)
]  # [x,y,size(radius)]

start = [0.0, 0.0, m.np.deg2rad(0.0)]
goal = [6.0, 7.0, m.np.deg2rad(90.0)]

def test2():
    step_size = 0.2
    rrt_star_reeds_shepp = m.RRTStarReedsShepp(start, goal,
                                             obstacleList, [-2.0, 15.0],
                                             max_iter=100, step_size=step_size)
    rrt_star_reeds_shepp.set_random_seed(seed=8)
    path = rrt_star_reeds_shepp.planning(animation=False)
    for i in range(len(path)-1):
        # + 0.00000000000001 for acceptable errors arising from the planning process
        assert m.math.dist(path[i][0:2], path[i+1][0:2]) < step_size + 0.00000000000001

def test_too_big_step_size():
    step_size = 20
    rrt_star_reeds_shepp = m.RRTStarReedsShepp(start, goal,
                                             obstacleList, [-2.0, 15.0],
                                             max_iter=100, step_size=step_size)
    rrt_star_reeds_shepp.set_random_seed(seed=8)
    path = rrt_star_reeds_shepp.planning(animation=False)
    assert path is None


def test_goal_yaw_wraparound():
    yaw_offset = 0.005
    planner = m.RRTStarReedsShepp(
        start=[0.0, 0.0, m.math.pi - yaw_offset],
        goal=[0.0, 0.0, -m.math.pi + yaw_offset],
        obstacle_list=[],
        rand_area=[-1.0, 1.0],
        max_iter=0,
    )
    planner.node_list = [planner.start]

    goal_index = planner.search_best_goal_node()

    assert goal_index == 0


if __name__ == '__main__':
    conftest.run_this_test(__file__)
