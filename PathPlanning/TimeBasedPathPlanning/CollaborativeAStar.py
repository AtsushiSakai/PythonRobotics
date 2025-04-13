"""
TODO - doc comment
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
from PathPlanning.TimeBasedPathPlanning.SpaceTimeAStar import SpaceTimeAStar
from PathPlanning.TimeBasedPathPlanning.Plotting import PlotNodePaths
import time

class CollaborativeAStar(MultiAgentPlanner):

    def plan(grid: Grid, start_and_goals: list[StartAndGoal], single_agent_planner_class: SingleAgentPlanner, verbose: bool) -> list[NodePath]: # TODO: list of what
        
        print(f"Using planner: {single_agent_planner_class}")

        # Reserve initial positions
        for start_and_goal in start_and_goals:
            grid.reserve_position(start_and_goal.start, start_and_goal.index, Interval(0, 10))

        # TODO: smarter ordering
        paths = []
        for start_and_goal in start_and_goals:
            if True:
                print(f"\nPlanning for agent:  {start_and_goal}" )

            grid.clear_initial_reservation(start_and_goal.start, start_and_goal.index)
            path = single_agent_planner_class.plan(grid, start_and_goal.start, start_and_goal.goal, verbose)

            if path is None:
                print(f"Failed to find path for {start_and_goal}")
                return []

            agent_index = start_and_goal.index
            grid.reserve_path(path, agent_index)
            paths.append(path)

        return paths

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
        obstacle_arrangement=ObstacleArrangement.NARROW_CORRIDOR,
        # obstacle_arrangement=ObstacleArrangement.ARRANGEMENT1,
        # obstacle_arrangement=ObstacleArrangement.RANDOM,
    )

    start_time = time.time()
    paths: list[NodePath] = CollaborativeAStar.plan(grid, start_and_goals, SafeIntervalPathPlanner, verbose)

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