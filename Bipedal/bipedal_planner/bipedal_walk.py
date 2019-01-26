"""
Bipedal Walking with modifying designated footsteps
Author: Takayuki Murooka (takayuki5168)
"""
from bipedal_planner import BipedalPlanner

def main():
    bipedal_planner = BipedalPlanner()
    
    footsteps = [[0.0, 0.2, 0.0],
                 [0.3, 0.2, 0.0],
                 [0.3, 0.2, 0.2],
                 [0.3, 0.2, 0.2],
                 [0.0, 0.2, 0.2]]
    bipedal_planner.set_ref_footsteps(footsteps)
    bipedal_planner.walk(plot=True)

if __name__ == "__main__":
    main()
