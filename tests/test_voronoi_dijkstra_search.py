import conftest  # Add root path to sys.path
from PathPlanning.VoronoiRoadMap.dijkstra_search import DijkstraSearch


def test_unreachable_goal_returns_empty_path():
    node_x = [0.0, 1.0, 2.0]
    node_y = [0.0, 0.0, 0.0]
    edge_ids_list = [[1], [0], []]

    rx, ry = DijkstraSearch(False).search(
        0.0, 0.0, 2.0, 0.0, node_x, node_y, edge_ids_list)

    assert rx == []
    assert ry == []


if __name__ == '__main__':
    conftest.run_this_test(__file__)
