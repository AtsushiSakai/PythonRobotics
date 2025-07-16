"""
Space-time A* Algorithm
    This script demonstrates the Space-time A* algorithm for path planning in a grid world with moving obstacles.
    This algorithm is different from normal 2D A* in one key way - the cost (often notated as g(n)) is
    the number of time steps it took to get to a given node, instead of the number of cells it has
    traversed. This ensures the path is time-optimal, while respecting any dynamic obstacles in the environment.

    Reference: https://www.davidsilver.uk/wp-content/uploads/2020/03/coop-path-AIWisdom.pdf
"""

import numpy as np
from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    ObstacleArrangement,
    Position,
)
from PathPlanning.TimeBasedPathPlanning.Node import Node, NodePath
import heapq
from collections.abc import Generator
import time
from PathPlanning.TimeBasedPathPlanning.BaseClasses import SingleAgentPlanner
from PathPlanning.TimeBasedPathPlanning.Plotting import PlotNodePath

class SpaceTimeAStar(SingleAgentPlanner):

    @staticmethod
    def plan(grid: Grid, start: Position, goal: Position, verbose: bool = False) -> NodePath:
        open_set: list[Node] = []
        heapq.heappush(
            open_set, Node(start, 0, SpaceTimeAStar.calculate_heuristic(start, goal), -1)
        )

        expanded_list: list[Node] = []
        expanded_set: set[Node] = set()
        while open_set:
            expanded_node: Node = heapq.heappop(open_set)
            if verbose:
                print("Expanded node:", expanded_node)

            if expanded_node.time + 1 >= grid.time_limit:
                if verbose:
                    print(f"\tSkipping node that is past time limit: {expanded_node}")
                continue

            if expanded_node.position == goal:
                if verbose:
                    print(f"Found path to goal after {len(expanded_list)} expansions")

                path = []
                path_walker: Node = expanded_node
                while True:
                    path.append(path_walker)
                    if path_walker.parent_index == -1:
                        break
                    path_walker = expanded_list[path_walker.parent_index]

                # reverse path so it goes start -> goal
                path.reverse()
                return NodePath(path, len(expanded_set))

            expanded_idx = len(expanded_list)
            expanded_list.append(expanded_node)
            expanded_set.add(expanded_node)

            for child in SpaceTimeAStar.generate_successors(grid, goal, expanded_node, expanded_idx, verbose, expanded_set):
                heapq.heappush(open_set, child)

        raise Exception("No path found")

    """
    Generate possible successors of the provided `parent_node`
    """
    @staticmethod
    def generate_successors(
        grid: Grid, goal: Position, parent_node: Node, parent_node_idx: int, verbose: bool, expanded_set: set[Node]
    ) -> Generator[Node, None, None]:
        diffs = [
            Position(0, 0),
            Position(1, 0),
            Position(-1, 0),
            Position(0, 1),
            Position(0, -1),
        ]
        for diff in diffs:
            new_pos = parent_node.position + diff
            new_node = Node(
                new_pos,
                parent_node.time + 1,
                SpaceTimeAStar.calculate_heuristic(new_pos, goal),
                parent_node_idx,
            )

            if new_node in expanded_set:
                continue

            # Check if the new node is valid for the next 2 time steps - one step to enter, and another to leave
            if all([grid.valid_position(new_pos, parent_node.time + dt) for dt in [1, 2]]):
                if verbose:
                    print("\tNew successor node: ", new_node)
                yield new_node

    @staticmethod
    def calculate_heuristic(position: Position, goal: Position) -> int:
        diff = goal - position
        return abs(diff.x) + abs(diff.y)


show_animation = True
verbose = False

def main():
    start = Position(1, 5)
    goal = Position(19, 19)
    grid_side_length = 21

    start_time = time.time()

    grid = Grid(
        np.array([grid_side_length, grid_side_length]),
        num_obstacles=40,
        obstacle_avoid_points=[start, goal],
        obstacle_arrangement=ObstacleArrangement.ARRANGEMENT1,
    )

    path = SpaceTimeAStar.plan(grid, start, goal, verbose)

    runtime = time.time() - start_time
    print(f"Planning took: {runtime:.5f} seconds")

    if verbose:
        print(f"Path: {path}")

    if not show_animation:
        return

    PlotNodePath(grid, start, goal, path)

if __name__ == "__main__":
    main()
