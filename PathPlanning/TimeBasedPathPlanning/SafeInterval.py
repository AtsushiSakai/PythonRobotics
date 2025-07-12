"""
Safe interval path planner
    This script implements a safe-interval path planner for a 2d grid with dynamic obstacles. It is faster than
    SpaceTime A* because it reduces the number of redundant node expansions by pre-computing regions of adjacent
    time steps that are safe ("safe intervals") at each position. This allows the algorithm to skip expanding nodes
    that are in intervals that have already been visited earlier.

    Reference: https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf
"""

import numpy as np
from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    Interval,
    ObstacleArrangement,
    Position,
    empty_2d_array_of_lists,
)
from PathPlanning.TimeBasedPathPlanning.BaseClasses import SingleAgentPlanner
from PathPlanning.TimeBasedPathPlanning.Node import Node, NodePath
from PathPlanning.TimeBasedPathPlanning.Plotting import PlotNodePath

import heapq
from dataclasses import dataclass
from functools import total_ordering
import time

@dataclass()
# Note: Total_ordering is used instead of adding `order=True` to the @dataclass decorator because
#     this class needs to override the __lt__ and __eq__ methods to ignore parent_index. The Parent
#     index and interval member variables are just used to track the path found by the algorithm,
#     and has no effect on the quality of a node.
@total_ordering
class SIPPNode(Node):
    interval: Interval

@dataclass
class EntryTimeAndInterval:
    entry_time: int
    interval: Interval

class SafeIntervalPathPlanner(SingleAgentPlanner):

    """
    Generate a plan given the loaded problem statement. Raises an exception if it fails to find a path.
    Arguments:
        verbose (bool): set to True to print debug information
    """
    @staticmethod
    def plan(grid: Grid, start: Position, goal: Position, verbose: bool = False) -> NodePath:

        safe_intervals = grid.get_safe_intervals()

        open_set: list[SIPPNode] = []
        first_node_interval = safe_intervals[start.x, start.y][0]
        heapq.heappush(
            open_set, SIPPNode(start, 0, SafeIntervalPathPlanner.calculate_heuristic(start, goal), -1, first_node_interval)
        )

        expanded_list: list[SIPPNode] = []
        visited_intervals = empty_2d_array_of_lists(grid.grid_size[0], grid.grid_size[1])
        while open_set:
            expanded_node: SIPPNode = heapq.heappop(open_set)
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
                path_walker: SIPPNode = expanded_node
                while True:
                    path.append(path_walker)
                    if path_walker.parent_index == -1:
                        break
                    path_walker = expanded_list[path_walker.parent_index]

                # reverse path so it goes start -> goal
                path.reverse()
                return NodePath(path, len(expanded_list))

            expanded_idx = len(expanded_list)
            expanded_list.append(expanded_node)
            entry_time_and_node = EntryTimeAndInterval(expanded_node.time, expanded_node.interval)
            add_entry_to_visited_intervals_array(entry_time_and_node, visited_intervals, expanded_node)

            for child in SafeIntervalPathPlanner.generate_successors(grid, goal, expanded_node, expanded_idx, safe_intervals, visited_intervals):
                heapq.heappush(open_set, child)

        raise Exception("No path found")

    """
    Generate list of possible successors of the provided `parent_node` that are worth expanding
    """
    @staticmethod
    def generate_successors(
        grid: Grid, goal: Position, parent_node: SIPPNode, parent_node_idx: int, intervals: np.ndarray, visited_intervals: np.ndarray
    ) -> list[SIPPNode]:
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
            if not grid.inside_grid_bounds(new_pos):
                continue

            current_interval = parent_node.interval

            new_cell_intervals: list[Interval] = intervals[new_pos.x, new_pos.y]
            for interval in new_cell_intervals:
                # if interval starts after current ends, break
                # assumption: intervals are sorted by start time, so all future intervals will hit this condition as well
                if interval.start_time > current_interval.end_time:
                    break

                # if interval ends before current starts, skip
                if interval.end_time < current_interval.start_time:
                    continue

                # if we have already expanded a node in this interval with a <= starting time, skip
                better_node_expanded = False
                for visited in visited_intervals[new_pos.x, new_pos.y]:
                    if interval == visited.interval and visited.entry_time <= parent_node.time + 1:
                        better_node_expanded = True
                        break
                if better_node_expanded:
                    continue

                # We know there is a node worth expanding. Generate successor at the earliest possible time the
                # new interval can be entered
                for possible_t in range(max(parent_node.time + 1, interval.start_time), min(current_interval.end_time, interval.end_time)):
                    if grid.valid_position(new_pos, possible_t):
                        new_nodes.append(SIPPNode(
                            new_pos,
                            # entry is max of interval start and parent node time + 1 (get there as soon as possible)
                            max(interval.start_time, parent_node.time + 1),
                            SafeIntervalPathPlanner.calculate_heuristic(new_pos, goal),
                            parent_node_idx,
                            interval,
                        ))
                        # break because all t's after this will make nodes with a higher cost, the same heuristic, and are in the same interval
                        break

        return new_nodes

    """
    Calculate the heuristic for a given position - Manhattan distance to the goal
    """
    @staticmethod
    def calculate_heuristic(position: Position, goal: Position) -> int:
        diff = goal - position
        return abs(diff.x) + abs(diff.y)


"""
Adds a new entry to the visited intervals array. If the entry is already present, the entry time is updated if the new
entry time is better. Otherwise, the entry is added to `visited_intervals` at the position of `expanded_node`.
"""
def add_entry_to_visited_intervals_array(entry_time_and_interval: EntryTimeAndInterval, visited_intervals: np.ndarray, expanded_node: SIPPNode):
    # if entry is present, update entry time if better
    for existing_entry_and_interval in visited_intervals[expanded_node.position.x, expanded_node.position.y]:
        if existing_entry_and_interval.interval == entry_time_and_interval.interval:
            existing_entry_and_interval.entry_time = min(existing_entry_and_interval.entry_time, entry_time_and_interval.entry_time)

    # Otherwise, append
    visited_intervals[expanded_node.position.x, expanded_node.position.y].append(entry_time_and_interval)


show_animation = True
verbose = False

def main():
    start = Position(1, 18)
    goal = Position(19, 19)
    grid_side_length = 21

    start_time = time.time()

    grid = Grid(
        np.array([grid_side_length, grid_side_length]),
        num_obstacles=250,
        obstacle_avoid_points=[start, goal],
        obstacle_arrangement=ObstacleArrangement.ARRANGEMENT1,
        # obstacle_arrangement=ObstacleArrangement.RANDOM,
    )

    path = SafeIntervalPathPlanner.plan(grid, start, goal, verbose)
    runtime = time.time() - start_time
    print(f"Planning took: {runtime:.5f} seconds")

    if verbose:
        print(f"Path: {path}")

    if not show_animation:
        return

    PlotNodePath(grid, start, goal, path)


if __name__ == "__main__":
    main()
