import conftest
from PathPlanning.InformedRRTStar import informed_rrt_star as m


def test1():
    m.show_animation = False
    m.main()


def test_equidistant_neighbors_keep_distinct_indices_and_cheaper_parent():
    planner = m.InformedRRTStar(
        start=[-1.0, 0.0], goal=[5.0, 0.0],
        obstacle_list=[], rand_area=[-5.0, 5.0]
    )
    left = m.Node(-1.0, 0.0)
    right = m.Node(1.0, 0.0)
    left.cost = 9.0
    right.cost = 1.0
    planner.node_list = [left, right]

    candidate = m.Node(0.0, 0.0)
    near_indices = planner.find_near_nodes(candidate)

    assert near_indices == [0, 1]
    chosen = planner.choose_parent(candidate, near_indices)
    assert chosen.parent == 1
    assert chosen.cost == 2.0


def test_near_nodes_preserve_equal_position_vertices():
    planner = m.InformedRRTStar(
        start=[0.0, 0.0], goal=[5.0, 0.0],
        obstacle_list=[], rand_area=[-5.0, 5.0]
    )
    planner.node_list = [
        m.Node(1.0, 1.0), m.Node(1.0, 1.0), m.Node(100.0, 100.0)
    ]

    assert planner.find_near_nodes(m.Node(0.0, 0.0)) == [0, 1]


if __name__ == '__main__':
    conftest.run_this_test(__file__)
