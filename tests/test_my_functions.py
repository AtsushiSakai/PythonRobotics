import conftest
import numpy as np
from coverageTracker import branch_coverage
from PathPlanning.RRT.sobol import sobol as s
from PathPlanning.ClosedLoopRRTStar import pure_pursuit as p

def print_coverage():
    for branch, hit in branch_coverage.items():
        print(f"{branch} was {'hit' if hit else 'not hit'}")




