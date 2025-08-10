from dataclasses import dataclass
from typing import Optional, TypeAlias
import heapq

from PathPlanning.TimeBasedPathPlanning.Node import NodePath, Position

AgentId: TypeAlias = int

@dataclass
class Constraint:
    position: Position
    time: int

@dataclass
class PathConstraint:
    constraint: Constraint
    shorter_path_agent: AgentId
    longer_path_agent: AgentId

@dataclass
class ConstraintTreeNode:
    parent_idx = int
    constraint: tuple[AgentId, Constraint]

    paths: dict[AgentId, NodePath]
    cost: int

    def __lt__(self, other) -> bool:
        # TODO - this feels jank?
        return self.cost + self.constrained_path_cost() < other.cost + other.constrained_path_cost()

    def get_constraint_point(self) -> Optional[PathConstraint]:
        final_t = max(path.goal_reached_time() for path in self.paths)
        positions_at_time: dict[Position, AgentId] = {}
        for t in range(final_t + 1):
            # TODO: need to be REALLY careful that these agent ids are consitent
            for agent_id, path in self.paths.items():
                position = path.get_position(t)
                if position is None:
                    continue
                if position in positions_at_time:
                    conflicting_agent_id = positions_at_time[position]
                    this_agent_shorter = self.paths[agent_id].goal_reached_time() < self.paths[conflicting_agent_id].goal_reached_time()

                    return PathConstraint(
                        constraint=Constraint(position=position, time=t),
                        shorter_path_agent= agent_id if this_agent_shorter else conflicting_agent_id,
                        longer_path_agent= conflicting_agent_id if this_agent_shorter else agent_id
                    )
        return None

    def constrained_path_cost(self) -> int:
        constrained_path = self.paths[self.constraint[0]]
        return constrained_path.goal_reached_time()

class ConstraintTree:
    # Child nodes have been created (Maps node_index to ConstraintTreeNode)
    expanded_nodes: dict[int, ConstraintTreeNode]
    # Need to solve and generate children
    nodes_to_expand: heapq[ConstraintTreeNode]

    solution: Optional[ConstraintTreeNode] = None

    def __init__(self, initial_solution: dict[AgentId, NodePath]):
        initial_cost = sum(path.goal_reached_time() for path in initial_solution.values())
        heapq.heappush(self.nodes_to_expand, ConstraintTreeNode(constraints={}, paths=initial_solution, cost=initial_cost, parent_idx=-1))

    def get_next_node_to_expand(self) -> Optional[ConstraintTreeNode]:
        if not self.nodes_to_expand:
            return None
        return heapq.heappop(self.nodes_to_expand)
    
    def add_node_to_tree(self, node: ConstraintTreeNode) -> bool:
        """
        Add a node to the tree and generate children if needed. Returns true if the node is a solution, false otherwise.
        """
        node_index = len(self.expanded_nodes)
        self.expanded_nodes[node_index] = node
        constraint_point = node.get_constraint_point()
        if constraint_point is None:
            # Don't need to add any constraints, this is a solution!
            self.solution = node
            return
        
        child_node1 = node
        child_node1.constraint = (constraint_point.shorter_path_agent, constraint_point.constraint)
        child_node1.parent_idx = node_index

        child_node2 = node
        child_node2.constraint = (constraint_point.longer_path_agent, constraint_point.constraint)
        child_node2.parent_idx = node_index

        heapq.heappush(self.nodes_to_expand, child_node1)
        heapq.heappush(self.nodes_to_expand, child_node2)
    
    def get_ancestor_constraints(self, parent_index: int):
        """
        Get the constraints that were applied to the parent node to generate this node.
        """
        constraints = []
        while parent_index != -1:
            node = self.expanded_nodes[parent_index]
            if node.constraint is not None:
                constraints.append(node.constraint)
            parent_index = node.parent_idx
        return constraints
