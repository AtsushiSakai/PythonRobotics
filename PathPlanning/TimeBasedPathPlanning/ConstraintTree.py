from dataclasses import dataclass
from typing import Optional, TypeAlias
import heapq

from PathPlanning.TimeBasedPathPlanning.Node import NodePath, Position, PositionAtTime

AgentId: TypeAlias = int

@dataclass(frozen=True)
class Constraint:
    position: Position
    time: int

@dataclass
class ForkingConstraint:
    constraint: Constraint
    constrained_agents: tuple[AgentId, AgentId]

@dataclass(frozen=True)
class AppliedConstraint:
    constraint: Constraint
    constrained_agent: AgentId

@dataclass
class ConstraintTreeNode:
    parent_idx = int
    constraint: Optional[ForkingConstraint | AppliedConstraint]

    paths: dict[AgentId, NodePath]
    cost: int

    def __init__(self, paths: dict[AgentId, NodePath], parent_idx: int):
        self.paths = paths
        self.cost = sum(path.goal_reached_time() for path in paths.values())
        self.parent_idx = parent_idx
        self.constraint = self.get_constraint_point()

    def __lt__(self, other) -> bool:
        return self.cost < other.cost

    def get_constraint_point(self, verbose = False) -> Optional[ForkingConstraint]:
        if verbose:
            print(f"\tpath for {agent_id}: {path}\n")

        final_t = max(path.goal_reached_time() for path in self.paths.values())
        positions_at_time: dict[PositionAtTime, AgentId] = {}
        for t in range(final_t + 1):
            # TODO: need to be REALLY careful that these agent ids are consitent
            for agent_id, path in self.paths.items():
                position = path.get_position(t)
                if position is None:
                    continue
                # print(f"reserving pos/t for {agent_id}: {position} @ {t}")
                position_at_time = PositionAtTime(position, t)
                if position_at_time in positions_at_time:
                    conflicting_agent_id = positions_at_time[position_at_time]

                    if verbose:
                        print(f"found constraint: {position_at_time} for agents {agent_id} & {conflicting_agent_id}")
                    return ForkingConstraint(
                        constraint=Constraint(position=position, time=t),
                        constrained_agents=(AgentId(agent_id), AgentId(conflicting_agent_id))
                    )
                else:
                    positions_at_time[position_at_time] = AgentId(agent_id)
        return None


class ConstraintTree:
    # Child nodes have been created (Maps node_index to ConstraintTreeNode)
    expanded_nodes: dict[int, ConstraintTreeNode]
    # Need to solve and generate children
    nodes_to_expand: heapq #[ConstraintTreeNode]

    def __init__(self, initial_solution: dict[AgentId, NodePath]):
        self.nodes_to_expand = []
        self.expanded_nodes = {}
        heapq.heappush(self.nodes_to_expand, ConstraintTreeNode(initial_solution, -1))

    def get_next_node_to_expand(self) -> Optional[ConstraintTreeNode]:
        if not self.nodes_to_expand:
            return None
        return heapq.heappop(self.nodes_to_expand)
    
    def add_node_to_tree(self, node: ConstraintTreeNode) -> bool:
        """
        Add a node to the tree and generate children if needed. Returns true if the node is a solution, false otherwise.
        """
        heapq.heappush(self.nodes_to_expand, node)
    
    def get_ancestor_constraints(self, parent_index: int) -> list[AppliedConstraint]:
        """
        Get the constraints that were applied to all parent nodes starting with the node at the provided parent_index.
        """
        constraints: list[AppliedConstraint] = []
        while parent_index != -1:
            node = self.expanded_nodes[parent_index]
            if node.constraint and isinstance(node.constraint, AppliedConstraint):
                constraints.append(node.constraint)
            else:
                print(f"Aha!!! {node.constraint}")
            parent_index = node.parent_idx
        return constraints

    def add_expanded_node(self, node: ConstraintTreeNode) -> int:
        """
        Add an expanded node to the tree. Returns the index of this node in the expanded nodes dictionary.
        """
        parent_idx = len(self.expanded_nodes)
        self.expanded_nodes[parent_idx] = node
        return parent_idx
