from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    NodePath,
    ObstacleArrangement,
    Position,
)
from PathPlanning.TimeBasedPathPlanning.BaseClasses import StartAndGoal
from PathPlanning.TimeBasedPathPlanning import ConflictBasedSearch as m
from PathPlanning.TimeBasedPathPlanning.SpaceTimeAStar import SpaceTimeAStar
from PathPlanning.TimeBasedPathPlanning.SafeInterval import SafeIntervalPathPlanner
from PathPlanning.TimeBasedPathPlanning.BaseClasses import SingleAgentPlanner
import numpy as np
import conftest
import pytest


@pytest.mark.parametrize("single_agent_planner", [SpaceTimeAStar, SafeIntervalPathPlanner])
def test_no_constraints(single_agent_planner):
    # Test that planner succeeds with no constraints
    grid_side_length = 21
    start_and_goals = [StartAndGoal(i, Position(1, 8+i), Position(19, 19-i)) for i in range(4)]

    obstacle_avoid_points = [pos for item in start_and_goals for pos in (item.start, item.goal)]

    grid = Grid(
        np.array([grid_side_length, grid_side_length]),
        num_obstacles=250,
        obstacle_avoid_points=obstacle_avoid_points,
        obstacle_arrangement=ObstacleArrangement.NONE,
    )

    paths: list[NodePath]
    paths = m.ConflictBasedSearch.plan(grid, start_and_goals, single_agent_planner, False)

    # All paths should start at the specified position and reach the goal
    for start_and_goal in start_and_goals:
        assert paths[start_and_goal.agent_id].path[0].position == start_and_goal.start
        assert paths[start_and_goal.agent_id].path[-1].position == start_and_goal.goal

@pytest.mark.parametrize("single_agent_planner", [SpaceTimeAStar, SafeIntervalPathPlanner])
def test_narrow_corridor(single_agent_planner):
    # Test a case that requires a few constraints
    grid_side_length = 21
    start_and_goals = [StartAndGoal(i, Position(1, 8+i), Position(19, 19-i)) for i in range(4)]

    obstacle_avoid_points = [pos for item in start_and_goals for pos in (item.start, item.goal)]

    grid = Grid(
        np.array([grid_side_length, grid_side_length]),
        num_obstacles=250,
        obstacle_avoid_points=obstacle_avoid_points,
        obstacle_arrangement=ObstacleArrangement.NARROW_CORRIDOR,
    )

    paths: list[NodePath]
    paths = m.ConflictBasedSearch.plan(grid, start_and_goals, single_agent_planner, False)

    # All paths should start at the specified position and reach the goal
    for start_and_goal in start_and_goals:
        assert paths[start_and_goal.agent_id].path[0].position == start_and_goal.start
        assert paths[start_and_goal.agent_id].path[-1].position == start_and_goal.goal

@pytest.mark.parametrize("single_agent_planner", [SpaceTimeAStar, SafeIntervalPathPlanner])
def test_hallway_pass(single_agent_planner: SingleAgentPlanner):
    # Test that search finds a path that requires a robot to temporarily move away from its goal
    grid_side_length = 21
    start_and_goals = [StartAndGoal(0, Position(6, 10), Position(13, 10)),
                       StartAndGoal(1, Position(11, 10), Position(6, 10)),
                       StartAndGoal(2, Position(13, 10), Position(7, 10))]
    obstacle_avoid_points = [pos for item in start_and_goals for pos in (item.start, item.goal)]

    grid = Grid(
        np.array([grid_side_length, grid_side_length]),
        num_obstacles=250,
        obstacle_avoid_points=obstacle_avoid_points,
        obstacle_arrangement=ObstacleArrangement.HALLWAY,
    )

    paths: list[NodePath]
    paths = m.ConflictBasedSearch.plan(grid, start_and_goals, single_agent_planner, False)

    # All paths should start at the specified position and reach the goal
    for start_and_goal in start_and_goals:
        assert paths[start_and_goal.agent_id].path[0].position == start_and_goal.start
        assert paths[start_and_goal.agent_id].path[-1].position == start_and_goal.goal

if __name__ == "__main__":
    m.show_animation = False
    conftest.run_this_test(__file__)
