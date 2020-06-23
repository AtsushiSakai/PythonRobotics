#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

sys.path.append('environment')
sys.path.append('samplingplanner')
 
import Environment as env
import RRTPlanner as rrt


if __name__ == "__main__":
    #define the planning scenarios
    #scenario name
    #start pose
    #goal pose
    #number of DOF to be solved (2, 3, 6)
    scenarios = [("environments/simple_test", [[1,0,0,2],[0,1,0,2],[0,0,1,0],[0,0,0,1]], [[1,0,0,-2],[0,1,0,-2],[0,0,1,0],[0,0,0,1]], 2, [(-3,3), (-3,3)]),
                 ("environments/simple_test", [[1,0,0,2],[0,1,0,2],[0,0,1,0],[0,0,0,1]], [[1,0,0,-2],[0,1,0,-2],[0,0,1,0],[0,0,0,1]], 3, [(-3,3), (-3,3)]),
("environments/alpha_puzzle", [[1,0,0,0],[0,1,0,5],[0,0,1,0],[0,0,0,1]], [[1,0,0,25],[0,1,0,25],[0,0,1,25],[0,0,0,1]], 6, [(-40,70),(-40,70),(-40,70)])]

    plt.ion()

    for scenario in scenarios:
        name = scenario[0]
        start = np.asarray(scenario[1])
        goal = np.asarray(scenario[2])
        dof = scenario[3]
        limits = scenario[4]
        
        print("processing scenario: " + name)

        #initiate environment and robot meshes
        environment = env.Environment()
        environment.load_environment(name)

        #instantiate the planner
        planner = rrt.RRTPlanner(dof, limits)
    
        #plan the path through the environment
        path = planner.plan(environment, start, goal)

        #plot the path step by step
        ax = None
        for Pose in path:
            ax = environment.plot_environment(Pose, ax=ax, limits=limits)
            plt.pause(1)
