import conftest
import numpy as np
from coverageTracker import branch_coverage
from PathPlanning.RRT.sobol import sobol as s
from PathPlanning.ClosedLoopRRTStar import pure_pursuit as p
from PathPlanning.ClosedLoopRRTStar import unicycle_model as u

def print_coverage():
    for branch, hit in branch_coverage.items():
        print(f"{branch} was {'hit' if hit else 'not hit'}")

def test_tau_sobol_dimNum_bigger_than_dimMax():
    dimNum = 10000
    assert s.tau_sobol(dimNum) == -1
    print_coverage()

def test_tau_sobol_dimNum_smaller_than_1():
    dimNum = -32323
    assert s.tau_sobol(dimNum) == -1
    print_coverage()

def test_tau_sobol_dimNum_valid_cases():
    tau_table = [0, 0, 1, 3, 5, 8, 11, 15, 19, 23, 27, 31, 35]
    for dimNum in range(1,13):
        assert s.tau_sobol(dimNum) == tau_table[dimNum]

def test_tau_sobol_dimNum_equal_to_zero():
    dimNum = 0
    assert s.tau_sobol(dimNum) == -1

def test_PIDControl_a_bigger():
    assert p.PIDControl(5, 1) == u.accel_max

def test_PIDControl_a_smaller():
    assert p.PIDControl(-100, 0) == -u.accel_max


if __name__ == '__main__':
    conftest.run_this_test(__file__)
