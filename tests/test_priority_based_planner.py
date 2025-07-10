from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    NodePath,
    ObstacleArrangement,
    Position,
)
from PathPlanning.TimeBasedPathPlanning.BaseClasses import StartAndGoal
from PathPlanning.TimeBasedPathPlanning import PriorityBasedPlanner as m
from PathPlanning.TimeBasedPathPlanning.SafeInterval import SafeIntervalPathPlanner
import numpy as np
import conftest


def test_1():
    grid_side_length = 21

    start_and_goals = [StartAndGoal(i, Position(1, i), Position(19, 19-i)) for i in range(1, 16)]
    obstacle_avoid_points = [pos for item in start_and_goals for pos in (item.start, item.goal)]

    grid = Grid(
        np.array([grid_side_length, grid_side_length]),
        num_obstacles=250,
        obstacle_avoid_points=obstacle_avoid_points,
        obstacle_arrangement=ObstacleArrangement.ARRANGEMENT1,
    )

    m.show_animation = False

    start_and_goals: list[StartAndGoal]
    paths: list[NodePath]
    start_and_goals, paths = m.PriorityBasedPlanner.plan(grid, start_and_goals, SafeIntervalPathPlanner, False)

    # All paths should start at the specified position and reach the goal
    for i, start_and_goal in enumerate(start_and_goals):
        assert paths[i].path[0].position == start_and_goal.start
        assert paths[i].path[-1].position == start_and_goal.goal

if __name__ == "__main__":
    conftest.run_this_test(__file__)
