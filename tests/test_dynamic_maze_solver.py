import conftest
from PathPlanning.Dynamic_maze_Solver.dynamic_maze_solver import MazeVisualizer


def test_bfs_finds_path():
    # small maze: 0=open, 1=wall
    maze = [
        [0, 0, 0],
        [1, 1, 0],
        [0, 0, 0]
    ]

    start = (0, 0)
    target = (2, 2)

    viz = MazeVisualizer(maze, start, target)

    path, visited = viz._bfs()

    assert path is not None
    assert path[0] == start
    assert path[-1] == target


if __name__ == '__main__':
    conftest.run_this_test(__file__)
