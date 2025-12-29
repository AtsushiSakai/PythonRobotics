"""
Conflict Based Search generates paths in 3 dimensions (x, y, time) for a set of agents. It does
so by performing searches on two levels. The top level search applies constraints that agents
must avoid, and the bottom level performs a single-agent search for individual agents given
the constraints provided by the top level search. Initially, paths are generated for each agent
with no constraints. The paths are checked for conflicts with one another. If any are found, the
top level search generates two nodes. These nodes apply a constraint at the point of conflict for
one of the two agents in conflict. This process repeats until a set of paths are found where no
agents are in conflict. The top level search chooses successor nodes based on the sum of the
cost of all paths, which guarantees optimiality of the final solution.

The full algorithm is defined in this paper: https://cdn.aaai.org/ojs/8140/8140-13-11667-1-2-20201228.pdf
"""

from copy import deepcopy
from enum import Enum
import numpy as np
from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    ObstacleArrangement,
    Position,
)
import time

from PathPlanning.TimeBasedPathPlanning.BaseClasses import MultiAgentPlanner, StartAndGoal
from PathPlanning.TimeBasedPathPlanning.Node import NodePath
from PathPlanning.TimeBasedPathPlanning.BaseClasses import SingleAgentPlanner
from PathPlanning.TimeBasedPathPlanning.SafeInterval import SafeIntervalPathPlanner
from PathPlanning.TimeBasedPathPlanning.SpaceTimeAStar import SpaceTimeAStar
from PathPlanning.TimeBasedPathPlanning.Plotting import PlotNodePaths
from PathPlanning.TimeBasedPathPlanning.ConstraintTree import AgentId, AppliedConstraint, ConstrainedAgent, ConstraintTree, ConstraintTreeNode, ForkingConstraint

class ConflictBasedSearch(MultiAgentPlanner):


    @staticmethod
    def plan(grid: Grid, start_and_goals: list[StartAndGoal], single_agent_planner_class: SingleAgentPlanner, verbose: bool = False) -> dict[AgentId, NodePath]:
        """
        Generate a path from the start to the goal for each agent in the `start_and_goals` list.
        """
        print(f"Using single-agent planner: {single_agent_planner_class}")

        initial_solution: dict[AgentId, NodePath] = {}

        # Generate initial solution (no constraints)
        for start_and_goal in start_and_goals:
            path = single_agent_planner_class.plan(grid, start_and_goal.start, start_and_goal.goal, start_and_goal.agent_id, verbose)
            initial_solution[start_and_goal.agent_id] = path

        if verbose:
            print("Initial solution:")
            for (agent_idx, path) in initial_solution.items():
                print(f"\nAgent {agent_idx} path:\n {path}")

        constraint_tree = ConstraintTree(initial_solution)
        attempted_constraint_combos: set = set()

        while constraint_tree.nodes_to_expand:
            constraint_tree_node = constraint_tree.get_next_node_to_expand()
            if constraint_tree_node is None:
                raise RuntimeError("No more nodes to expand in the constraint tree.")

            ancestor_constraints = constraint_tree.get_ancestor_constraints(constraint_tree_node.parent_idx)

            if verbose:
                print(f"Expanding node with constraint {constraint_tree_node.constraint} and parent {constraint_tree_node.parent_idx}")
                print(f"\tCOST: {constraint_tree_node.cost}")

            if constraint_tree_node.constraint is None:
                raise RuntimeError("No more nodes to expand in the constraint tree.")
            if not constraint_tree_node.constraint:
                # This means we found a solution!
                print(f"\nFound a path with constraints after {constraint_tree.expanded_node_count()} expansions")
                print(f"Final cost: {constraint_tree_node.cost}")
                print(f"Number of constraints on solution: {len(constraint_tree_node.all_constraints)}")
                final_paths = {}
                for start_and_goal in start_and_goals:
                    final_paths[start_and_goal.agent_id] = constraint_tree_node.paths[start_and_goal.agent_id]
                return final_paths

            if not isinstance(constraint_tree_node.constraint, ForkingConstraint):
                raise ValueError(f"Expected a ForkingConstraint, but got: {constraint_tree_node.constraint}")

            for constrained_agent in constraint_tree_node.constraint.constrained_agents:
                applied_constraint = AppliedConstraint(constrained_agent.constraint, constrained_agent.agent)

                all_constraints = ancestor_constraints
                all_constraints.append(applied_constraint)

                new_path = ConflictBasedSearch.plan_for_agent(
                    constrained_agent,
                    all_constraints,
                    constraint_tree,
                    attempted_constraint_combos,
                    grid,
                    single_agent_planner_class,
                    start_and_goals,
                    verbose
                )
                if not new_path:
                    continue

                # Deepcopy to update with applied constraint and new paths
                applied_constraint_parent = deepcopy(constraint_tree_node)
                applied_constraint_parent.paths[constrained_agent.agent] = new_path

                if verbose:
                    for (agent_idx, path) in applied_constraint_parent.paths.items():
                        print(f"\nAgent {agent_idx} path:\n {path}")

                applied_constraint_parent.constraint = applied_constraint
                parent_idx = constraint_tree.add_expanded_node(applied_constraint_parent)

                new_constraint_tree_node = ConstraintTreeNode(applied_constraint_parent.paths, parent_idx, all_constraints)
                if verbose:
                    print(f"Adding new constraint tree node with constraint: {new_constraint_tree_node.constraint}")
                constraint_tree.add_node_to_tree(new_constraint_tree_node)

        raise RuntimeError("No solution found")

    @staticmethod
    def get_agents_start_and_goal(start_and_goal_list: list[StartAndGoal], target_index: AgentId) -> StartAndGoal:
        """
        Returns the start and goal of a specific agent
        """
        for item in start_and_goal_list:
            if item.agent_id == target_index:
                return item
        raise RuntimeError(f"Could not find agent with index {target_index} in {start_and_goal_list}")

    @staticmethod
    def plan_for_agent(constrained_agent: ConstrainedAgent,
                 all_constraints: list[AppliedConstraint],
                 constraint_tree: ConstraintTree,
                 attempted_constraint_combos: set,
                 grid: Grid,
                 single_agent_planner_class: SingleAgentPlanner,
                 start_and_goals: list[StartAndGoal],
                 verbose: bool) -> NodePath | None:
        """
        Attempt to generate a path plan for a single agent
        """
        num_expansions = constraint_tree.expanded_node_count()
        if num_expansions % 50 == 0:
            print(f"Expanded {num_expansions} nodes so far...")
            print(f"\tNumber of constraints on expanded node: {len(all_constraints)}")

        # Skip if we have already tried this set of constraints
        constraint_hash = hash(frozenset(all_constraints))
        if constraint_hash in attempted_constraint_combos:
            if verbose:
                print(f"\tSkipping already attempted constraint combination: {all_constraints}")
            return None
        else:
            attempted_constraint_combos.add(constraint_hash)

        if verbose:
            print(f"\tall constraints: {all_constraints}")

        grid.clear_constraint_points()
        grid.apply_constraint_points(all_constraints)

        # Just plan for agent with new constraint
        start_and_goal: StartAndGoal = ConflictBasedSearch.get_agents_start_and_goal(start_and_goals, constrained_agent.agent)
        try:
            if verbose:
                print("\tplanning for: {}", start_and_goal)
            new_path = single_agent_planner_class.plan(grid, start_and_goal.start, start_and_goal.goal, start_and_goal.agent_id, verbose)
            return new_path
        except Exception as e:
            if verbose:
                print(f"Error: {e}")
            return None

class Scenario(Enum):
    # Five robots all trying to get through a single cell choke point to reach their goals
    NARROW_CORRIDOR = 0
    # Three robots in a narrow hallway that requires intelligent conflict resolution
    HALLWAY_CROSS = 1

scenario = Scenario.HALLWAY_CROSS
verbose = False
show_animation = True
use_sipp = True # Condition here mainly to appease the linter
np.random.seed(42)  # For reproducibility
def main():
    grid_side_length = 21

    # Default: no obstacles
    obstacle_arrangement = ObstacleArrangement.NONE
    start_and_goals = [StartAndGoal(i, Position(1, i), Position(19, 19-i)) for i in range(10)]

    if scenario == Scenario.NARROW_CORRIDOR:
        obstacle_arrangement=ObstacleArrangement.NARROW_CORRIDOR
        start_and_goals = [StartAndGoal(i, Position(1, 8+i), Position(19, 19-i)) for i in range(5)]
    elif scenario == Scenario.HALLWAY_CROSS:
        obstacle_arrangement=ObstacleArrangement.HALLWAY
        start_and_goals = [StartAndGoal(0, Position(6, 10), Position(13, 10)),
                           StartAndGoal(1, Position(11, 10), Position(6, 10)),
                           StartAndGoal(2, Position(13, 10), Position(7, 10))]

    grid = Grid(
        np.array([grid_side_length, grid_side_length]),
        num_obstacles=250,
        obstacle_avoid_points=[pos for item in start_and_goals for pos in (item.start, item.goal)],
        obstacle_arrangement=obstacle_arrangement,
    )

    single_agent_planner = SafeIntervalPathPlanner if use_sipp else SpaceTimeAStar
    start_time = time.time()
    paths = ConflictBasedSearch.plan(grid, start_and_goals, single_agent_planner, verbose)

    runtime = time.time() - start_time
    print(f"\nPlanning took: {runtime:.5f} seconds")

    if verbose:
        print("Paths:")
        for path in paths.values():
            print(f"{path}\n")

    if not show_animation:
        return

    PlotNodePaths(grid, start_and_goals, paths)

if __name__ == "__main__":
    main()
