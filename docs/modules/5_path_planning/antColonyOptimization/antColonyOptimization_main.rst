Ant Colony Optimization Path Planning
--------------------------------------

This is a grid-based path planning implementation using Ant Colony Optimization (ACO).

ACO is a metaheuristic optimization algorithm inspired by the foraging behavior of real ants. Ants deposit pheromones on paths they traverse, and other ants are attracted to paths with higher pheromone concentrations. Over time, shorter paths accumulate more pheromones, leading the colony to converge on optimal solutions.

Code Link
~~~~~~~~~

.. autofunction:: PathPlanning.AntColonyOptimization.ant_colony_optimization.main

Algorithm Overview
~~~~~~~~~~~~~~~~~~

The ACO algorithm simulates a colony of ants exploring the search space:

1. **Initialization**: Initialize pheromone trails to small positive values
2. **Solution Construction**: Each ant probabilistically constructs a path from start to goal
3. **Pheromone Update**: Ants deposit pheromones on their paths (shorter paths get more)
4. **Evaporation**: All pheromone trails decay by a constant factor
5. **Iteration**: Repeat until convergence or maximum iterations reached

Animation
~~~~~~~~~

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/AntColonyOptimization/animation.gif

Mathematical Foundation
~~~~~~~~~~~~~~~~~~~~~~~

The probability of ant k moving from node i to node j is:

.. math::

   P_{ij}^k = \frac{[\tau_{ij}]^\alpha \cdot [\eta_{ij}]^\beta}{\sum_{l \in N_i^k} [\tau_{il}]^\alpha \cdot [\eta_{il}]^\beta}

Where:

- :math:`\tau_{ij}` = pheromone level on edge (i,j)
- :math:`\eta_{ij}` = heuristic value (inverse of distance to goal)
- :math:`\alpha` = pheromone importance factor
- :math:`\beta` = heuristic importance factor
- :math:`N_i^k` = set of feasible neighbors of node i for ant k

Pheromone Update Rule
~~~~~~~~~~~~~~~~~~~~~

.. math::

   \tau_{ij}(t+1) = (1-\rho) \cdot \tau_{ij}(t) + \sum_{k=1}^{m} \Delta\tau_{ij}^k

Where:

- :math:`\rho` = evaporation rate (0 < :math:`\rho` < 1)
- :math:`\Delta\tau_{ij}^k = Q / L_k` if ant k used edge (i,j), else 0
- :math:`Q` = pheromone deposit constant
- :math:`L_k` = path length of ant k

Advantages
~~~~~~~~~~

- **Adaptive search**: Pheromones guide exploration toward promising regions
- **Distributed computation**: Multiple ants search in parallel
- **Positive feedback**: Good solutions reinforce themselves
- **Memory**: Pheromone trails encode search history
- **Flexibility**: Can handle dynamic environments

Disadvantages
~~~~~~~~~~~~~

- **Stochastic**: Results may vary between runs
- **Slow convergence**: May require many iterations
- **Parameter sensitivity**: Performance depends on alpha, beta, rho, Q
- **Memory intensive**: Stores pheromone matrix for entire grid
- **Premature convergence**: May stagnate if parameters not tuned well

References
~~~~~~~~~

- `Ant colony optimization algorithms - Wikipedia <https://en.wikipedia.org/wiki/Ant_colony_optimization_algorithms>`__
- Dorigo, M.; Maniezzo, V.; Colorni, A. (1996). "Ant system: optimization by a colony of cooperating agents". IEEE Transactions on Systems, Man, and Cybernetics, Part B. 26 (1): 29–41.
- Dorigo, M.; Stützle, T. (2004). "Ant Colony Optimization". MIT Press.
