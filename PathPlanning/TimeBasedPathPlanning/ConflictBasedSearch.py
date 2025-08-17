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

        # Generate initial solution (no reservations for robots)
        for start_and_goal in start_and_goals:
            path = single_agent_planner_class.plan(grid, start_and_goal.start, start_and_goal.goal, start_and_goal.index, verbose)
            initial_solution[AgentId(start_and_goal.index)] = path

        for (agent_idx, path) in initial_solution.items():
            print(f"\nAgent {agent_idx} path:\n {path}")

        constraint_tree = ConstraintTree(initial_solution)

        attempted_constraint_combos = set()

        while constraint_tree.nodes_to_expand:
            constraint_tree_node = constraint_tree.get_next_node_to_expand()
            ancestor_constraints = constraint_tree.get_ancestor_constraints(constraint_tree_node.parent_idx)

            print(f"Expanded node: {constraint_tree_node.constraint} with parent: {constraint_tree_node.parent_idx}")
            print(f"\tAncestor constraints: {ancestor_constraints}")

            if verbose:
                print(f"Expanding node with constraint {constraint_tree_node.constraint} and parent {constraint_tree_node.parent_idx}")

            if constraint_tree_node is None:
                raise RuntimeError("No more nodes to expand in the constraint tree.")
            if not constraint_tree_node.constraint:
                # This means we found a solution!
                return (start_and_goals, [constraint_tree_node.paths[start_and_goal.index] for start_and_goal in start_and_goals])

            if not isinstance(constraint_tree_node.constraint, ForkingConstraint):
                raise ValueError(f"Expected a ForkingConstraint, but got: {constraint_tree_node.constraint}")

            # TODO: contents of this loop should probably be in a helper?
            for constrained_agent in constraint_tree_node.constraint.constrained_agents:
                if verbose:
                    print(f"\nOuter loop step for agent {constrained_agent}")

                applied_constraint = AppliedConstraint(constrained_agent.constraint, constrained_agent.agent)

                # TODO: check type of other constraints somewhere
                all_constraints = deepcopy(ancestor_constraints) # TODO - no deepcopy pls
                all_constraints.append(applied_constraint)

                # Skip if we have already tried this set of constraints
                constraint_hash = hash(frozenset(all_constraints))
                if constraint_hash in attempted_constraint_combos:
                    print(f"\tSkipping already attempted constraint combination: {all_constraints}")
                    continue
                else:
                    attempted_constraint_combos.add(constraint_hash)

                if verbose:
                    print(f"\tall constraints: {all_constraints}")
                print(f"\tall constraints: {all_constraints}")

                grid.clear_constraint_points()
                grid.apply_constraint_points(all_constraints)

                # Just plan for agent with new constraint
                start_and_goal = ConflictBasedSearch.find_by_index(start_and_goals, constrained_agent.agent)
                try:
                    print("\tplanning for: {}", start_and_goal)
                    new_path = single_agent_planner_class.plan(grid, start_and_goal.start, start_and_goal.goal, start_and_goal.index, verbose)
                except Exception as e:
                    continue

                applied_constraint_parent = deepcopy(constraint_tree_node) #TODO: not sure if deepcopy is actually needed
                paths: dict[AgentId, NodePath] = deepcopy(applied_constraint_parent.paths) # TODO: not sure if deepcopy is actually needed
                paths[constrained_agent.agent] = new_path

                for (agent_idx, path) in paths.items():
                    print(f"\nAgent {agent_idx} path:\n {path}")

                applied_constraint_parent.constraint = applied_constraint
                parent_idx = constraint_tree.add_expanded_node(applied_constraint_parent)

                new_constraint_tree_node = ConstraintTreeNode(deepcopy(paths), parent_idx, all_constraints)
                if new_constraint_tree_node.constraint is None:
                    # This means we found a solution!
                    print("Found a path with constraints:")
                    for constraint in all_constraints:
                        print(f"\t{constraint}")
                    # return (start_and_goals, [paths[AgentId(i)] for i in range(len(start_and_goals))])
                    return (start_and_goals, paths.values())

                # if verbose:
                # print(f"Adding new constraint tree node with constraint: {new_constraint_tree_node.constraint}")
                constraint_tree.add_node_to_tree(new_constraint_tree_node)

        raise RuntimeError("No solution found")
    
    # TODO: bad function name
    def find_by_index(start_and_goal_list: list[StartAndGoal], target_index: AgentId) -> AgentId:
        for item in start_and_goal_list:
            if item.index == target_index:
                return item
        raise RuntimeError(f"Could not find agent with index {target_index} in {start_and_goal_list}")

# TODO
# * get SIPP working w CBS
# * add checks for duplicate expansions
# * fan out across multiple threads
# * sipp intervals seem much larger than needed?

verbose = False
show_animation = True
def main():
    grid_side_length = 21

    # TODO: bug somewhere where it expects agent ids to match indices
    # start_and_goals = [StartAndGoal(i, Position(1, i), Position(19, 19-i)) for i in range(1, 12)]
    # start_and_goals = [StartAndGoal(i, Position(1, 8+i), Position(19, 19-i)) for i in range(6)]
    # start_and_goals = [StartAndGoal(i, Position(1, 2*i), Position(19, 19-i)) for i in range(4)]

    # hallway cross
    start_and_goals = [StartAndGoal(0, Position(6, 10), Position(13, 10)),
                       StartAndGoal(1, Position(11, 10), Position(6, 10)),
                       StartAndGoal(2, Position(13, 10), Position(7, 10))]

    # temporary obstacle
    # start_and_goals = [StartAndGoal(0, Position(15, 14), Position(15, 16))]

    # start_and_goals = [StartAndGoal(1, Position(6, 10), Position(8, 10)),
    #                    StartAndGoal(2, Position(13, 10), Position(11, 10))]
    obstacle_avoid_points = [pos for item in start_and_goals for pos in (item.start, item.goal)]

    grid = Grid(
        np.array([grid_side_length, grid_side_length]),
        num_obstacles=250,
        obstacle_avoid_points=obstacle_avoid_points,
        # obstacle_arrangement=ObstacleArrangement.TEMPORARY_OBSTACLE,
        obstacle_arrangement=ObstacleArrangement.HALLWAY,
        # obstacle_arrangement=ObstacleArrangement.NARROW_CORRIDOR,
        # obstacle_arrangement=ObstacleArrangement.ARRANGEMENT1,
        # obstacle_arrangement=ObstacleArrangement.RANDOM,
    )

    start_time = time.time()
    # start_and_goals, paths = ConflictBasedSearch.plan(grid, start_and_goals, SafeIntervalPathPlanner, verbose)
    start_and_goals, paths = ConflictBasedSearch.plan(grid, start_and_goals, SpaceTimeAStar, verbose)

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