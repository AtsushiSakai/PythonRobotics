from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    NodePath,
    ObstacleArrangement,
    Position,
)
from PathPlanning.TimeBasedPathPlanning.BaseClasses import StartAndGoal
from PathPlanning.TimeBasedPathPlanning import PriorityBasedPlanner as m
from PathPlanning.TimeBasedPathPlanning.SpaceTimeAStar import SpaceTimeAStar
from PathPlanning.TimeBasedPathPlanning.SafeInterval import SafeIntervalPathPlanner
from PathPlanning.TimeBasedPathPlanning.BaseClasses import SingleAgentPlanner
import numpy as np
import conftest
import pytest

@pytest.mark.parametrize("single_agent_planner", [SpaceTimeAStar, SafeIntervalPathPlanner])
def test_1(single_agent_planner: SingleAgentPlanner):
    grid_side_length = 21

    start_and_goals = [StartAndGoal(i, Position(1, i), Position(19, 19-i)) for i in range(1, 11)]
    obstacle_avoid_points = [pos for item in start_and_goals for pos in (item.start, item.goal)]

    grid = Grid(
        np.array([grid_side_length, grid_side_length]),
        num_obstacles=250,
        obstacle_avoid_points=obstacle_avoid_points,
        obstacle_arrangement=ObstacleArrangement.ARRANGEMENT1,
    )

    m.show_animation = False

    paths: list[NodePath]
    paths = m.PriorityBasedPlanner.plan(grid, start_and_goals, single_agent_planner, False)

    # All paths should start at the specified position and reach the goal
    for start_and_goal in start_and_goals:
        assert paths[start_and_goal.agent_id].path[0].position == start_and_goal.start
        assert paths[start_and_goal.agent_id].path[-1].position == start_and_goal.goal

if __name__ == "__main__":
    conftest.run_this_test(__file__)
