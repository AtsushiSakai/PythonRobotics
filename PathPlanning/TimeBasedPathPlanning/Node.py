from dataclasses import dataclass
from functools import total_ordering
import numpy as np
from typing import Sequence

@dataclass(order=True)
class Position:
    x: int
    y: int

    def as_ndarray(self) -> np.ndarray:
        return np.array([self.x, self.y])

    def __add__(self, other):
        if isinstance(other, Position):
            return Position(self.x + other.x, self.y + other.y)
        raise NotImplementedError(
            f"Addition not supported for Position and {type(other)}"
        )

    def __sub__(self, other):
        if isinstance(other, Position):
            return Position(self.x - other.x, self.y - other.y)
        raise NotImplementedError(
            f"Subtraction not supported for Position and {type(other)}"
        )

    def __hash__(self):
        return hash((self.x, self.y))

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
    path: Sequence[Node]
    positions_at_time: dict[int, Position]
    # Number of nodes expanded while finding this path
    expanded_node_count: int

    def __init__(self, path: Sequence[Node], expanded_node_count: int):
        self.path = path
        self.expanded_node_count = expanded_node_count

        self.positions_at_time = {}
        for i, node in enumerate(path):
            reservation_finish_time = node.time + 1
            if i < len(path) - 1:
                reservation_finish_time = path[i + 1].time

            for t in range(node.time, reservation_finish_time):
                self.positions_at_time[t] = node.position

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