from dataclasses import dataclass
from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Position,
)
from functools import total_ordering

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