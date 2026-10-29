import conftest  # Add root path to sys.path
import numpy as np
from scipy.spatial import KDTree
from PathPlanning.ProbabilisticRoadMap import probabilistic_road_map


def test1():
    probabilistic_road_map.show_animation = False
    probabilistic_road_map.main(rng=np.random.default_rng(1233))



def test_collision_detects_obstacle_between_edge_samples():
    # The obstacle is less than 1 m from the segment, but more than
    # 1 m from both endpoints (the old discrete samples).
    obstacle_tree = KDTree([[0.5, 0.95]])
    assert probabilistic_road_map.is_collision(
        0.0, 0.0, 1.0, 0.0, 1.0, obstacle_tree
    )
    assert not probabilistic_road_map.is_collision(
        0.0, 0.0, 1.0, 0.0, 0.9, obstacle_tree
    )


def test_collision_with_point_robot():
    on_edge = KDTree([[0.5, 0.0]])
    off_edge = KDTree([[0.5, 0.01]])

    assert probabilistic_road_map.is_collision(
        0.0, 0.0, 1.0, 0.0, 0.0, on_edge
    )
    assert not probabilistic_road_map.is_collision(
        0.0, 0.0, 1.0, 0.0, 0.0, off_edge
    )


def test_collision_handles_zero_length_and_endpoint_clearance():
    start_obstacle = KDTree([[0.0, 0.0]])
    outside_segment = KDTree([[2.0, 0.0]])

    assert probabilistic_road_map.is_collision(
        0.0, 0.0, 0.0, 0.0, 0.0, start_obstacle
    )
    assert not probabilistic_road_map.is_collision(
        0.0, 0.0, 0.0, 0.0, 0.0, outside_segment
    )
    assert not probabilistic_road_map.is_collision(
        0.0, 0.0, 1.0, 0.0, 0.6, outside_segment
    )


if __name__ == '__main__':
    conftest.run_this_test(__file__)
