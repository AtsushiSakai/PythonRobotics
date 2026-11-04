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


def _planner_for_tied_distances(obstacles=None):
    return m.RRTStar(start=[0., -2.], goal=[0., 0.],
                     obstacle_list=[] if obstacles is None else obstacles,
                     rand_area=[-2., 2.], expand_dis=2., path_resolution=0.05)


def test_near_nodes_preserves_distinct_equidistant_vertices():
    planner = _planner_for_tied_distances()
    planner.node_list = [planner.Node(-1., 0.), planner.Node(1., 0.),
                         planner.Node(0., -1.), planner.Node(0., 1.),
                         planner.Node(3., 0.)]
    assert planner.find_near_nodes(planner.Node(0., 0.)) == [0, 1, 2, 3]


def test_parent_selection_considers_all_equidistant_vertices():
    planner = _planner_for_tied_distances()
    expensive = planner.Node(-1., 0.)
    expensive.cost = 10.
    cheaper = planner.Node(1., 0.)
    cheaper.cost = 3.
    planner.node_list = [expensive, cheaper]
    target = planner.Node(0., 0.)
    parented = planner.choose_parent(target, planner.find_near_nodes(target))
    assert parented.parent is cheaper
    assert parented.cost == 4.


def test_goal_selection_considers_cheaper_equidistant_vertex():
    planner = _planner_for_tied_distances()
    first = planner.Node(-1., 0.)
    first.cost = 10.
    second = planner.Node(1., 0.)
    second.cost = 3.
    planner.node_list = [first, second]
    assert planner.search_best_goal_node() == 1


def test_goal_selection_keeps_collision_free_equidistant_vertex():
    planner = _planner_for_tied_distances(obstacles=[(-0.5, 0., 0.1)])
    first = planner.Node(-1., 0.)
    first.cost = 3.
    second = planner.Node(1., 0.)
    second.cost = 4.
    planner.node_list = [first, second]
    first_edge = planner.steer(first, planner.goal_node)
    second_edge = planner.steer(second, planner.goal_node)
    assert not planner.check_collision(first_edge, planner.obstacle_list, 0.)
    assert planner.check_collision(second_edge, planner.obstacle_list, 0.)
    assert planner.search_best_goal_node() == 1


def test_goal_selection_preserves_same_position_vertices():
    planner = _planner_for_tied_distances()
    first = planner.Node(1., 0.)
    first.cost = 10.
    second = planner.Node(1., 0.)
    second.cost = 3.
    planner.node_list = [first, second]
    assert planner.search_best_goal_node() == 1


if __name__ == '__main__':
    conftest.run_this_test(__file__)
