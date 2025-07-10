from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    ObstacleArrangement,
    Position,
)
from PathPlanning.TimeBasedPathPlanning import SafeInterval as m
import numpy as np
import conftest


def test_1():
    start = Position(1, 11)
    goal = Position(19, 19)
    grid_side_length = 21
    grid = Grid(
        np.array([grid_side_length, grid_side_length]),
        obstacle_arrangement=ObstacleArrangement.ARRANGEMENT1,
    )

    m.show_animation = False
    path = m.SafeIntervalPathPlanner.plan(grid, start, goal)

    # path should have 31 entries
    assert len(path.path) == 31

    # path should end at the goal
    assert path.path[-1].position == goal


if __name__ == "__main__":
    conftest.run_this_test(__file__)
