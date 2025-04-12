from PathPlanning.TimeBasedPathPlanning.GridWithDynamicObstacles import (
    Grid,
    Position,
)
from PathPlanning.TimeBasedPathPlanning.Node import NodePath
import random
import numpy.random as numpy_random
from abc import ABC, abstractmethod

class SingleAgentPlanner(ABC):
    """
    Base class for single agent path planners.
    """

    def __init__(self, grid: Grid, start: Position, goal: Position):
        self.grid = grid
        self.start = start
        self.goal = goal

        # Seed randomness for reproducibility
        RANDOM_SEED = 50
        random.seed(RANDOM_SEED)
        numpy_random.seed(RANDOM_SEED)
    
    @abstractmethod
    def plan(self, verbose: bool = False) -> NodePath:
        pass
