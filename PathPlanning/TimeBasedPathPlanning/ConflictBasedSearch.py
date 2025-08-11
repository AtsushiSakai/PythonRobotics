"""
TODO - docstring

Algorithm is defined in this paper: https://cdn.aaai.org/ojs/8140/8140-13-11667-1-2-20201228.pdf
"""

import numpy as np
from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    ObstacleArrangement,
    Position,
)
from copy import deepcopy
from PathPlanning.TimeBasedPathPlanning.BaseClasses import MultiAgentPlanner, StartAndGoal
from PathPlanning.TimeBasedPathPlanning.Node import NodePath
from PathPlanning.TimeBasedPathPlanning.BaseClasses import SingleAgentPlanner
from PathPlanning.TimeBasedPathPlanning.SafeInterval import SafeIntervalPathPlanner
from PathPlanning.TimeBasedPathPlanning.SpaceTimeAStar import SpaceTimeAStar
from PathPlanning.TimeBasedPathPlanning.Plotting import PlotNodePaths
from PathPlanning.TimeBasedPathPlanning.ConstraintTree import AgentId, AppliedConstraint, ConstraintTree, ConstraintTreeNode, ForkingConstraint
import time

class ConflictBasedSearch(MultiAgentPlanner):

    @staticmethod
    def plan(grid: Grid, start_and_goals: list[StartAndGoal], single_agent_planner_class: SingleAgentPlanner, verbose: bool = False) -> tuple[list[StartAndGoal], list[NodePath]]:
        """
        Generate a path from the start to the goal for each agent in the `start_and_goals` list.
        Returns the re-ordered StartAndGoal combinations, and a list of path plans. The order of the plans
        corresponds to the order of the `start_and_goals` list.
        """
        print(f"Using single-agent planner: {single_agent_planner_class}")

        initial_solution: dict[AgentId, NodePath] = {}

        # Reserve initial positions
        for agent_idx, start_and_goal in enumerate(start_and_goals):
            # grid.reserve_position(start_and_goal.start, start_and_goal.index, Interval(0, 10))
            path = single_agent_planner_class.plan(grid, start_and_goal.start, start_and_goal.goal, agent_idx, verbose)
            initial_solution[AgentId(agent_idx)] = path

        constraint_tree = ConstraintTree(initial_solution)

        while constraint_tree.nodes_to_expand:
            constraint_tree_node = constraint_tree.get_next_node_to_expand()
            ancestor_constraints = constraint_tree.get_ancestor_constraints(constraint_tree_node.parent_idx)
            print(f"Expanded node: {constraint_tree_node.constraint} with parent: {constraint_tree_node.parent_idx}")
            print(f"\tAncestor constraints: {ancestor_constraints}")

            if verbose:
                print(f"Expanding node with constraint {constraint_tree_node.constraint} and parent {constraint_tree_node.parent_idx} ")

            if constraint_tree_node is None:
                raise RuntimeError("No more nodes to expand in the constraint tree.")
            if not constraint_tree_node.constraint:
                # This means we found a solution!
                return (start_and_goals, [constraint_tree_node.paths[AgentId(i)] for i in range(len(start_and_goals))])

            if not isinstance(constraint_tree_node.constraint, ForkingConstraint):
                raise ValueError(f"Expected a ForkingConstraint, but got: {constraint_tree_node.constraint}")

            # TODO: contents of this loop should probably be in a helper?
            for constrained_agent in constraint_tree_node.constraint.constrained_agents:
                paths: dict[AgentId, NodePath] = {}

                if verbose:
                    print(f"\nOuter loop step for agent {constrained_agent}")

                applied_constraint = AppliedConstraint(constraint_tree_node.constraint.constraint, constrained_agent)
                all_constraints = deepcopy(ancestor_constraints)
                all_constraints.append(applied_constraint)

                if verbose:
                    print(f"\tall constraints: {all_constraints}")

                grid.clear_constraint_points()
                grid.apply_constraint_points(all_constraints)

                for agent_idx, start_and_goal in enumerate(start_and_goals):
                    path = single_agent_planner_class.plan(grid, start_and_goal.start, start_and_goal.goal, agent_idx, verbose)
                    if path is None:
                        raise RuntimeError(f"Failed to find path for {start_and_goal}")
                    paths[AgentId(start_and_goal.index)] = path

                applied_constraint_parent = deepcopy(constraint_tree_node) #TODO: not sure if deepcopy is actually needed
                applied_constraint_parent.constraint = applied_constraint
                parent_idx = constraint_tree.add_expanded_node(applied_constraint_parent)

                new_constraint_tree_node = ConstraintTreeNode(paths, parent_idx)
                if new_constraint_tree_node.constraint is None:
                    # This means we found a solution!
                    return (start_and_goals, [paths[AgentId(i)] for i in range(len(start_and_goals))])

                if verbose:
                    print(f"Adding new constraint tree node with constraint: {new_constraint_tree_node.constraint}")
                constraint_tree.add_node_to_tree(new_constraint_tree_node)

verbose = False
show_animation = True
def main():
    grid_side_length = 21

    # start_and_goals = [StartAndGoal(i, Position(1, i), Position(19, 19-i)) for i in range(1, 16)]
    start_and_goals = [StartAndGoal(i, Position(1, 8+i), Position(19, 19-i)) for i in range(4)]
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
    start_and_goals, paths = ConflictBasedSearch.plan(grid, start_and_goals, SafeIntervalPathPlanner, verbose)
    # start_and_goals, paths = ConflictBasedSearch.plan(grid, start_and_goals, SpaceTimeAStar, verbose)

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