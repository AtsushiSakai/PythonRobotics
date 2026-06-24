import conftest
from PathPlanning.DynamicMazeSolver import dynamic_maze_solver as m


def test_bfs_finds_path():
    # small maze: 0=open, 1=wall
    maze = [
        [0, 0, 0],
        [1, 1, 0],
        [0, 0, 0]
    ]

    start = (0, 0)
    target = (2, 2)

    # module `dynamic_maze_solver` exposes `MazeVisualizer` class
    viz = m.MazeVisualizer(maze, start, target)

    path, visited = viz._bfs()

    assert path is not None
    assert path[0] == start
    assert path[-1] == target


def test_bfs_unreachable_target():
    # target is enclosed by walls
    maze = [
        [0, 1, 0],
        [1, 1, 1],
        [0, 1, 0]
    ]

    start = (0, 0)
    target = (2, 2)

    viz = m.MazeVisualizer(maze, start, target)

    path, visited = viz._bfs()

    assert path is None
    assert target not in visited


def test_bfs_start_equals_target():
    # trivial case where start == target
    maze = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ]

    start = (1, 1)
    target = start

    viz = m.MazeVisualizer(maze, start, target)

    path, visited = viz._bfs()

    assert path is not None
    assert path == [start]


if __name__ == '__main__':
    conftest.run_this_test(__file__)
