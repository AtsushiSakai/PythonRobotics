from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    Position,
)
from PathPlanning.TimeBasedPathPlanning.Node import NodePath
import random
import numpy.random as numpy_random
from abc import ABC, abstractmethod
from dataclasses import dataclass

class SingleAgentPlanner(ABC):
    """
    Base class for single agent planners
    """
    
    @staticmethod
    @abstractmethod
    def plan(grid: Grid, start: Position, goal: Position, verbose: bool = False) -> NodePath:
        pass

@dataclass
class StartAndGoal:
    # Start position of the robot
    start: Position
    # Goal position of the robot
    goal: Position

class MultiAgentPlanner(ABC):
    """
    Base class for multi-agent planners
    """

    def __init__(self, grid: Grid, start_and_goal_positions: list[StartAndGoal]):
        self.grid = grid
        self.start_and_goal_positions = start_and_goal_positions

        # Seed randomness for reproducibility
        RANDOM_SEED = 50
        random.seed(RANDOM_SEED)
        numpy_random.seed(RANDOM_SEED)
    
    def plan(self, verbose: bool = False) -> list[NodePath]:
        """
        Plan for all agents. Returned paths are in the order of the `StartAndGoal` list this object was instantiated with
        """
        paths = []
        for start_and_goal in self.start_and_goal_positions:
            planner = self.create_planner(start_and_goal.start, start_and_goal.goal)
            path = planner.plan(verbose)
            paths.append(path)
        return paths