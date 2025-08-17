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
class ConstrainedAgent:
    agent: AgentId
    constraint: Constraint

@dataclass
class ForkingConstraint:
    constrained_agents: tuple[ConstrainedAgent, ConstrainedAgent]

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

    def __init__(self, paths: dict[AgentId, NodePath], parent_idx: int, all_constraints: list[AppliedConstraint]):
        self.paths = paths
        self.cost = sum(path.goal_reached_time() for path in paths.values())
        self.parent_idx = parent_idx
        self.constraint = self.get_constraint_point()
        self.all_constraints = all_constraints

    def __lt__(self, other) -> bool:
        if self.cost == other.cost:
            return len(self.all_constraints) < len(other.all_constraints)
        return self.cost < other.cost

    def get_constraint_point(self, verbose = False) -> Optional[ForkingConstraint]:

        final_t = max(path.goal_reached_time() for path in self.paths.values())
        positions_at_time: dict[PositionAtTime, AgentId] = {}
        for t in range(final_t + 1):
            # TODO: need to be REALLY careful that these agent ids are consitent
            possible_constraints: list[ForkingConstraint] = []
            for agent_id, path in self.paths.items():
                # Check for edge conflicts
                last_position = None
                if t > 0:
                    last_position = path.get_position(t - 1)

                position = path.get_position(t)
                if position is None:
                    continue
                # print(f"\treserving pos/t for {agent_id}: {position} @ {t}")
                position_at_time = PositionAtTime(position, t)
                if position_at_time not in positions_at_time:
                    positions_at_time[position_at_time] = AgentId(agent_id)
                
                # edge conflict
                if last_position:
                    new_position_at_last_time = PositionAtTime(position, t-1)
                    old_position_at_new_time = PositionAtTime(last_position, t)
                    if new_position_at_last_time in positions_at_time and old_position_at_new_time in positions_at_time:
                        conflicting_agent_id1 = positions_at_time[new_position_at_last_time]
                        conflicting_agent_id2 = positions_at_time[old_position_at_new_time]
                        
                        if conflicting_agent_id1 == conflicting_agent_id2 and conflicting_agent_id1 != agent_id:
                            # print(f"Found edge constraint between with agent {conflicting_agent_id1} for {agent_id}")
                            # print(f"\tpositions old: {old_position_at_new_time}, new: {position_at_time}")
                            new_constraint = ForkingConstraint((
                                ConstrainedAgent(agent_id, position_at_time),
                                ConstrainedAgent(conflicting_agent_id1, Constraint(position=last_position, time=t))
                            ))
                            possible_constraints.append(new_constraint)
                            continue
                
                # double reservation at a (cell, time) combination
                if positions_at_time[position_at_time] != agent_id:
                    conflicting_agent_id = positions_at_time[position_at_time]

                    constraint = Constraint(position=position, time=t)
                    possible_constraints.append(ForkingConstraint((
                        ConstrainedAgent(agent_id, constraint), ConstrainedAgent(conflicting_agent_id, constraint)
                    )))
                    continue
            if possible_constraints:
                if verbose:
                    print(f"Choosing best constraint of {possible_constraints}")
                # first check for edge constraints
                for constraint in possible_constraints:
                    if constraint.constrained_agents[0].constraint.position != constraint.constrained_agents[1].constraint.position:
                        if verbose:
                            print(f"\tFound edge conflict constraint: {constraint}")
                        return constraint
                # if none, then return first normal constraint
                if verbose:
                    print(f"\tReturning normal constraint: {possible_constraints[0]}")
                return possible_constraints[0]

        return None


class ConstraintTree:
    # Child nodes have been created (Maps node_index to ConstraintTreeNode)
    expanded_nodes: dict[int, ConstraintTreeNode]
    # Need to solve and generate children
    nodes_to_expand: heapq #[ConstraintTreeNode]

    def __init__(self, initial_solution: dict[AgentId, NodePath]):
        self.nodes_to_expand = []
        self.expanded_nodes = {}
        heapq.heappush(self.nodes_to_expand, ConstraintTreeNode(initial_solution, -1, []))

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
                raise RuntimeError(f"Expected AppliedConstraint, but got: {node.constraint}")
            parent_index = node.parent_idx
        return constraints

    def add_expanded_node(self, node: ConstraintTreeNode) -> int:
        """
        Add an expanded node to the tree. Returns the index of this node in the expanded nodes dictionary.
        """
        parent_idx = len(self.expanded_nodes)
        self.expanded_nodes[parent_idx] = node
        return parent_idx

    def expanded_node_count(self) -> int:
        return len(self.expanded_nodes)