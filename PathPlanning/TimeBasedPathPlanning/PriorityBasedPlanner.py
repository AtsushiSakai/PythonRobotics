"""
Priority Based Planner for multi agent path planning.
The planner generates an order to plan in, and generates plans for the robots in that order. Each planned
path is reserved in the grid, and all future plans must avoid that path.

Algorithm outlined in section III of this paper: https://pure.tudelft.nl/ws/portalfiles/portal/67074672/07138650.pdf
"""

import numpy as np
from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    Interval,
    ObstacleArrangement,
    Position,
)
from PathPlanning.TimeBasedPathPlanning.BaseClasses import MultiAgentPlanner, StartAndGoal
from PathPlanning.TimeBasedPathPlanning.Node import NodePath
from PathPlanning.TimeBasedPathPlanning.BaseClasses import SingleAgentPlanner
from PathPlanning.TimeBasedPathPlanning.SafeInterval import SafeIntervalPathPlanner
from PathPlanning.TimeBasedPathPlanning.Plotting import PlotNodePaths
import time

class PriorityBasedPlanner(MultiAgentPlanner):

    @staticmethod
    def plan(grid: Grid, start_and_goals: list[StartAndGoal], single_agent_planner_class: SingleAgentPlanner, verbose: bool = False) -> tuple[list[StartAndGoal], list[NodePath]]:
        """
        Generate a path from the start to the goal for each agent in the `start_and_goals` list.
        Returns the re-ordered StartAndGoal combinations, and a list of path plans. The order of the plans
        corresponds to the order of the `start_and_goals` list.
        """
        print(f"Using single-agent planner: {single_agent_planner_class}")

        # Reserve initial positions
        for start_and_goal in start_and_goals:
            grid.reserve_position(start_and_goal.start, start_and_goal.index, Interval(0, 10))

        # Plan in descending order of distance from start to goal
        start_and_goals = sorted(start_and_goals,
                    key=lambda item: item.distance_start_to_goal(),
                    reverse=True)

        paths = []
        for start_and_goal in start_and_goals:
            if verbose:
                print(f"\nPlanning for agent:  {start_and_goal}" )

            grid.clear_initial_reservation(start_and_goal.start, start_and_goal.index)
            path = single_agent_planner_class.plan(grid, start_and_goal.start, start_and_goal.goal, verbose)

            if path is None:
                print(f"Failed to find path for {start_and_goal}")
                return []

            agent_index = start_and_goal.index
            grid.reserve_path(path, agent_index)
            paths.append(path)

        return (start_and_goals, paths)

verbose = False
show_animation = True
def main():
    grid_side_length = 21

    start_and_goals = [StartAndGoal(i, Position(1, i), Position(19, 19-i)) for i in range(1, 16)]
    obstacle_avoid_points = [pos for item in start_and_goals for pos in (item.start, item.goal)]

    grid = Grid(
        np.array([grid_side_length, grid_side_length]),
        num_obstacles=250,
        obstacle_avoid_points=obstacle_avoid_points,
        # obstacle_arrangement=ObstacleArrangement.NARROW_CORRIDOR,
        obstacle_arrangement=ObstacleArrangement.ARRANGEMENT1,
        # obstacle_arrangement=ObstacleArrangement.RANDOM,
    )

    start_time = time.time()
    start_and_goals, paths = PriorityBasedPlanner.plan(grid, start_and_goals, SafeIntervalPathPlanner, verbose)

    runtime = time.time() - start_time
    print(f"\nPlanning took: {runtime:.5f} seconds")

    if verbose:
        print(f"Paths:")
        for path in paths:
            print(f"{path}\n")

    if not show_animation:
        return

    PlotNodePaths(grid, start_and_goals, paths)

if __name__ == "__main__":
    main()