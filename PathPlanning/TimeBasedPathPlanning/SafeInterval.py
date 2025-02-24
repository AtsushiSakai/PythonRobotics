"""
Safe interval path planner

TODO: populate docstring
"""

import numpy as np
import matplotlib.pyplot as plt
from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    Interval,
    ObstacleArrangement,
    Position,
)
import heapq
from collections.abc import Generator
import random
from dataclasses import dataclass
from functools import total_ordering


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
    interval: Interval

    """
    This is what is used to drive node expansion. The node with the lowest value is expanded next.
    This comparison prioritizes the node with the lowest cost-to-come (self.time) + cost-to-go (self.heuristic)
    """
    def __lt__(self, other: object):
        if not isinstance(other, Node):
            return NotImplementedError(f"Cannot compare Node with object of type: {type(other)}")
        # TODO: assumption that these two carry all the info needed for intervals. I think that makes sense but should think about it
        return (self.time + self.heuristic) < (other.time + other.heuristic)

    """
    TODO - note about interval being included here
    """
    def __eq__(self, other: object):
        if not isinstance(other, Node):
            return NotImplementedError(f"Cannot compare Node with object of type: {type(other)}")
        return self.position == other.position and self.time == other.time and self.interval == other.interval


class NodePath:
    path: list[Node]
    positions_at_time: dict[int, Position] = {}

    def __init__(self, path: list[Node]):
        self.path = path
        for (i, node) in enumerate(path):
            if i > 0:
                # account for waiting in interval at previous node
                prev_node = path[i-1]
                for t in range(prev_node.time, node.time):
                    self.positions_at_time[t] = prev_node.position

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


class SafeIntervalPathPlanner:
    grid: Grid
    start: Position
    goal: Position

    def __init__(self, grid: Grid, start: Position, goal: Position):
        self.grid = grid
        self.start = start
        self.goal = goal

    def plan(self, verbose: bool = False) -> NodePath:

        safe_intervals = self.grid.get_safe_intervals()

        open_set: list[Node] = []
        first_node_interval = safe_intervals[self.start.x, self.start.y][0]
        heapq.heappush(
            open_set, Node(self.start, 0, self.calculate_heuristic(self.start), -1, first_node_interval)
        )

        expanded_list: list[Node] = []
        # TODO: copy pasta from Grid file
        # 2d np array of lists of (entry time, interval tuples)
        # TODO: use a dataclass for the tuple
        visited_intervals = np.empty((self.grid.grid_size[0], self.grid.grid_size[1]), dtype=object)
        visited_intervals[:] = [[[] for _ in range(visited_intervals.shape[1])] for _ in range(visited_intervals.shape[0])]
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
                while path_walker.parent_index != -1:
                    path.append(path_walker)
                    path_walker = expanded_list[path_walker.parent_index]
                # TODO: fix hack around bad while condiiotn
                path.append(path_walker)

                # reverse path so it goes start -> goal
                path.reverse()
                return NodePath(path)

            expanded_idx = len(expanded_list)
            expanded_list.append(expanded_node)
            visited_intervals[expanded_node.position.x, expanded_node.position.y].append((expanded_node.time, expanded_node.interval))

            # if len(expanded_set) > 100:
            #     blarg

            for child in self.generate_successors(expanded_node, expanded_idx, verbose, safe_intervals, visited_intervals):
                heapq.heappush(open_set, child)

        raise Exception("No path found")

    """
    Generate possible successors of the provided `parent_node`
    """
    # TODO: is intervals being passed by ref? (i think so?)
    def generate_successors(
        self, parent_node: Node, parent_node_idx: int, verbose: bool, intervals: np.ndarray, visited_intervals: np.ndarray
    ) -> list[Node]:
        new_nodes = []
        diffs = [
            Position(0, 0),
            Position(1, 0),
            Position(-1, 0),
            Position(0, 1),
            Position(0, -1),
        ]
        for diff in diffs:
            new_pos = parent_node.position + diff
            if not self.grid.inside_grid_bounds(new_pos):
                continue

            current_interval = parent_node.interval

            new_cell_intervals: list[Interval] = intervals[new_pos.x, new_pos.y]
            for interval in new_cell_intervals:
                # if interval ends before current starts, skip
                if interval.end_time < current_interval.start_time:
                    continue

                # if interval starts after current ends, break
                # TODO: assumption here that intervals are sorted (they should be)
                if interval.start_time > current_interval.end_time:
                    break

                # TODO: this bit feels wonky
                # if we have already expanded a node in this interval with a <= starting time, continue
                better_node_expanded = False
                for visited in visited_intervals[new_pos.x, new_pos.y]:
                    if interval == visited[1] and visited[0] <= parent_node.time + 1:
                        better_node_expanded = True
                        break
                if better_node_expanded:
                    continue

                # We know there is some overlap. Generate successor at the earliest possible time the
                # new interval can be entered
                # TODO: dont love the optionl usage here
                new_node_t = None
                for possible_t in range(max(parent_node.time + 1, interval.start_time), min(current_interval.end_time, interval.end_time)):
                    if self.grid.valid_position(new_pos, possible_t):
                        new_node_t = possible_t
                        break

                if new_node_t:
                    # TODO: should be able to break here?
                    new_nodes.append(Node(
                        new_pos,
                        # entry is max of interval start and parent node start time (get there as soon as possible)
                        max(parent_node.time + 1, interval.start_time),
                        self.calculate_heuristic(new_pos),
                        parent_node_idx,
                        interval,
                    ))

        return new_nodes

    def calculate_heuristic(self, position) -> int:
        diff = self.goal - position
        return abs(diff.x) + abs(diff.y)


show_animation = True
verbose = False

def main():
    start = Position(1, 1)
    goal = Position(19, 19)
    grid_side_length = 21
    grid = Grid(
        np.array([grid_side_length, grid_side_length]),
        num_obstacles=250,
        obstacle_avoid_points=[start, goal],
        # obstacle_arrangement=ObstacleArrangement.ARRANGEMENT1,
        obstacle_arrangement=ObstacleArrangement.RANDOM,
    )

    planner = SafeIntervalPathPlanner(grid, start, goal)
    path = planner.plan(verbose)

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

    for i in range(0, path.goal_reached_time() + 1):
        obs_positions = grid.get_obstacle_positions_at_time(i)
        obs_points.set_data(obs_positions[0], obs_positions[1])
        path_position = path.get_position(i)
        path_points.set_data([path_position.x], [path_position.y])
        plt.pause(0.2)
    plt.show()


if __name__ == "__main__":
    main()
