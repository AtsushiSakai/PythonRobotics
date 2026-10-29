import conftest  # Add root path to sys.path
from scipy.spatial import cKDTree

from PathPlanning.VoronoiRoadMap import voronoi_road_map as m


def test_edge_collision_detects_obstacle_between_samples():
    planner = m.VoronoiRoadMapPlanner()
    # The endpoints are each more than 1 m from the obstacle, but
    # the segment passes within 0.95 m of it.
    obstacle_tree = cKDTree([[0.5, 0.95]])

    assert planner.is_collision(0.0, 0.0, 1.0, 0.0, 1.0, obstacle_tree)
    assert not planner.is_collision(0.0, 0.0, 1.0, 0.0, 0.9, obstacle_tree)


def test_point_robot_detects_obstacles_on_the_segment():
    planner = m.VoronoiRoadMapPlanner()
    on_edge = cKDTree([[0.5, 0.0]])
    off_edge = cKDTree([[0.5, 0.01]])

    assert planner.is_collision(0.0, 0.0, 1.0, 0.0, 0.0, on_edge)
    assert not planner.is_collision(0.0, 0.0, 1.0, 0.0, 0.0, off_edge)


def test_zero_length_edge_checks_its_single_position():
    planner = m.VoronoiRoadMapPlanner()
    at_start = cKDTree([[0.0, 0.0]])
    far_away = cKDTree([[2.0, 0.0]])

    assert planner.is_collision(0.0, 0.0, 0.0, 0.0, 0.0, at_start)
    assert not planner.is_collision(0.0, 0.0, 0.0, 0.0, 0.0, far_away)


def test_edge_limit_and_obstacle_past_endpoint():
    planner = m.VoronoiRoadMapPlanner()
    far_away = cKDTree([[2.0, 0.0]])

    assert not planner.is_collision(0.0, 0.0, 1.0, 0.0, 0.6, far_away)
    assert planner.is_collision(
        0.0, 0.0, planner.MAX_EDGE_LEN, 0.0, 0.0, far_away
    )


if __name__ == '__main__':
    conftest.run_this_test(__file__)
