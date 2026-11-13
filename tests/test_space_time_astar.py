from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    ObstacleArrangement,
    Position,
)
from PathPlanning.TimeBasedPathPlanning import SpaceTimeAStar as m
import numpy as np
import pytest
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
    path = m.SpaceTimeAStar.plan(grid, start, goal)

    # path should have 28 entries
    assert len(path.path) == 31

    # path should end at the goal
    assert path.path[-1].position == goal

    assert path.expanded_node_count < 1000

def test_no_path_within_time_limit():
    # Goal is farther away than the time limit allows, so planning should
    # fail cleanly instead of indexing past the reservation matrix
    start = Position(1, 1)
    goal = Position(9, 9)
    grid = Grid(
        np.array([11, 11]),
        num_obstacles=0,
        obstacle_avoid_points=[start, goal],
        time_limit=5,
    )

    m.show_animation = False
    with pytest.raises(Exception, match="No path found"):
        m.SpaceTimeAStar.plan(grid, start, goal)

if __name__ == "__main__":
    conftest.run_this_test(__file__)
