"""
Space-time A* Algorithm
    This script demonstrates the Space-time A* algorithm for path planning in a grid world with moving obstacles.
    This algorithm is different from normal 2D A* in one key way - the cost (often notated as g(n)) is
    the number of time steps it took to get to a given node, instead of the number of cells it has
    traversed. This ensures the path is time-optimal, while respecting any dynamic obstacles in the environment.

    Reference: https://www.davidsilver.uk/wp-content/uploads/2020/03/coop-path-AIWisdom.pdf
"""

import numpy as np
import matplotlib.pyplot as plt
from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    ObstacleArrangement,
    Position,
)
import heapq
from collections.abc import Generator
import random
from dataclasses import dataclass
from functools import total_ordering
import time

# Seed randomness for reproducibility
RANDOM_SEED = 50
random.seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)

@dataclass()
# Note: Total_ordering is used instead of adding `order=True` to the @dataclass decorator because
#     this class needs to override the __lt__ and __eq__ methods to ignore parent_index. Parent
#     index is just used to track the path found by the algorithm, and has no effect on the quality
#     of a node.
@total_ordering
class Node:
    position: Position
    time: int
    heuristic: int
    parent_index: int

    """
    This is what is used to drive node expansion. The node with the lowest value is expanded next.
    This comparison prioritizes the node with the lowest cost-to-come (self.time) + cost-to-go (self.heuristic)
    """
    def __lt__(self, other: object):
        if not isinstance(other, Node):
            return NotImplementedError(f"Cannot compare Node with object of type: {type(other)}")
        return (self.time + self.heuristic) < (other.time + other.heuristic)

    """
    Note: cost and heuristic are not included in eq or hash, since they will always be the same
          for a given (position, time) pair. Including either cost or heuristic would be redundant.
    """
    def __eq__(self, other: object):
        if not isinstance(other, Node):
            return NotImplementedError(f"Cannot compare Node with object of type: {type(other)}")
        return self.position == other.position and self.time == other.time

    def __hash__(self):
        return hash((self.position, self.time))

class NodePath:
    path: list[Node]
    positions_at_time: dict[int, Position] = {}

    def __init__(self, path: list[Node]):
        self.path = path
        for node in path:
            self.positions_at_time[node.time] = node.position

    """
    Get the position of the path at a given time
    """
    def get_position(self, time: int) -> Position | None:
        return self.positions_at_time.get(time)

    """
    Time stamp of the last node in the path
    """
    def goal_reached_time(self) -> int:
        return self.path[-1].time

    def __repr__(self):
        repr_string = ""
        for i, node in enumerate(self.path):
            repr_string += f"{i}: {node}\n"
        return repr_string


class SpaceTimeAStar:
    grid: Grid
    start: Position
    goal: Position
    # Used to evaluate solutions
    expanded_node_count: int = -1

    def __init__(self, grid: Grid, start: Position, goal: Position):
        self.grid = grid
        self.start = start
        self.goal = goal

    def plan(self, verbose: bool = False) -> NodePath:
        open_set: list[Node] = []
        heapq.heappush(
            open_set, Node(self.start, 0, self.calculate_heuristic(self.start), -1)
        )

        expanded_list: list[Node] = []
        expanded_set: set[Node] = set()
        while open_set:
            expanded_node: Node = heapq.heappop(open_set)
            if verbose:
                print("Expanded node:", expanded_node)

            if expanded_node.time + 1 >= self.grid.time_limit:
                if verbose:
                    print(f"\tSkipping node that is past time limit: {expanded_node}")
                continue

            if expanded_node.position == self.goal:
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
                self.expanded_node_count = len(expanded_set)
                return NodePath(path)

            expanded_idx = len(expanded_list)
            expanded_list.append(expanded_node)
            expanded_set.add(expanded_node)

            for child in self.generate_successors(expanded_node, expanded_idx, verbose, expanded_set):
                heapq.heappush(open_set, child)

        raise Exception("No path found")

    """
    Generate possible successors of the provided `parent_node`
    """
    def generate_successors(
        self, parent_node: Node, parent_node_idx: int, verbose: bool, expanded_set: set[Node]
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
                self.calculate_heuristic(new_pos),
                parent_node_idx,
            )

            if new_node in expanded_set:
                continue

            if self.grid.valid_position(new_pos, parent_node.time + 1):
                if verbose:
                    print("\tNew successor node: ", new_node)
                yield new_node

    def calculate_heuristic(self, position) -> int:
        diff = self.goal - position
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

    planner = SpaceTimeAStar(grid, start, goal)
    path = planner.plan(verbose)

    runtime = time.time() - start_time
    print(f"Planning took: {runtime:.5f} seconds")

    if verbose:
        print(f"Path: {path}")

    if not show_animation:
        return

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(
        autoscale_on=False,
        xlim=(0, grid.grid_size[0] - 1),
        ylim=(0, grid.grid_size[1] - 1),
    )
    ax.set_aspect("equal")
    ax.grid()
    ax.set_xticks(np.arange(0, grid_side_length, 1))
    ax.set_yticks(np.arange(0, grid_side_length, 1))

    (start_and_goal,) = ax.plot([], [], "mD", ms=15, label="Start and Goal")
    start_and_goal.set_data([start.x, goal.x], [start.y, goal.y])
    (obs_points,) = ax.plot([], [], "ro", ms=15, label="Obstacles")
    (path_points,) = ax.plot([], [], "bo", ms=10, label="Path Found")
    ax.legend(bbox_to_anchor=(1.05, 1))

    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect(
        "key_release_event", lambda event: [exit(0) if event.key == "escape" else None]
    )

    for i in range(0, path.goal_reached_time()):
        obs_positions = grid.get_obstacle_positions_at_time(i)
        obs_points.set_data(obs_positions[0], obs_positions[1])
        path_position = path.get_position(i)
        path_points.set_data([path_position.x], [path_position.y])
        plt.pause(0.2)
    plt.show()


if __name__ == "__main__":
    main()
