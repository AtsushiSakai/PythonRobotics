branch_coverage_sobol = {
    "sobol_1": False, # if branch if the value of dim_num is within the inclusive range of 1 to dim_max
    "sobol_2": False, # if it is not
}

branch_coverage_pid = {
    "pure_pursuit_1": False, # if branch if it is bigger than the accel_max
    "pure_pursuit_2": False # if it is smaller than the accel_max
}