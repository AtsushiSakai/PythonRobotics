.. _particle_swarm_optimization:

Particle Swarm Optimization Path Planning
------------------------------------------

This is a 2D path planning simulation using the Particle Swarm Optimization algorithm.

PSO is a metaheuristic optimization algorithm inspired by the social behavior of bird flocking or fish schooling. In path planning, particles represent potential solutions that explore the search space to find collision-free paths from start to goal.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ParticleSwarmOptimization/animation.gif

Algorithm Overview
++++++++++++++++++

The PSO algorithm maintains a swarm of particles that move through the search space according to simple mathematical rules:

1. **Initialization**: Particles are randomly distributed near the start position
2. **Evaluation**: Each particle's fitness is calculated based on distance to goal and obstacle penalties
3. **Update**: Particles adjust their velocities based on:
   - Personal best position (cognitive component)
   - Global best position (social component)  
   - Current velocity (inertia component)
4. **Movement**: Particles move to new positions and check for collisions
5. **Convergence**: Process repeats until maximum iterations or goal is reached

Mathematical Foundation
+++++++++++++++++++++++

The core PSO velocity update equation is:

.. math::

   v_{i}(t+1) = w \cdot v_{i}(t) + c_1 \cdot r_1 \cdot (p_{i} - x_{i}(t)) + c_2 \cdot r_2 \cdot (g - x_{i}(t))

Where:
- :math:`v_{i}(t)` = velocity of particle i at time t
- :math:`x_{i}(t)` = position of particle i at time t  
- :math:`w` = inertia weight (controls exploration vs exploitation)
- :math:`c_1` = cognitive coefficient (attraction to personal best)
- :math:`c_2` = social coefficient (attraction to global best)
- :math:`r_1, r_2` = random numbers in [0,1]
- :math:`p_{i}` = personal best position of particle i
- :math:`g` = global best position

Position update:

.. math::

   x_{i}(t+1) = x_{i}(t) + v_{i}(t+1)

Fitness Function
++++++++++++++++

The fitness function combines distance to target with obstacle penalties:

.. math::

   f(x) = ||x - x_{goal}|| + \sum_{j} P_{obs}(x, O_j)

Where:
- :math:`||x - x_{goal}||` = Euclidean distance to goal
- :math:`P_{obs}(x, O_j)` = penalty for obstacle j
- :math:`O_j` = obstacle j with position and radius

The obstacle penalty function is defined as:

.. math::

   P_{obs}(x, O_j) = \begin{cases}
   1000 & \text{if } ||x - O_j|| < r_j \text{ (inside obstacle)} \\
   \frac{50}{||x - O_j|| - r_j + 0.1} & \text{if } r_j \leq ||x - O_j|| < r_j + R_{influence} \text{ (near obstacle)} \\
   0 & \text{if } ||x - O_j|| \geq r_j + R_{influence} \text{ (safe distance)}
   \end{cases}

Where:
- :math:`r_j` = radius of obstacle j
- :math:`R_{influence}` = influence radius (typically 5 units)

Collision Detection
+++++++++++++++++++

Line-circle intersection is used to detect collisions between particle paths and circular obstacles:

.. math::

   ||P_0 + t \cdot \vec{d} - C|| = r

Where:
- :math:`P_0` = start point of path segment
- :math:`\vec{d}` = direction vector of path
- :math:`C` = obstacle center
- :math:`r` = obstacle radius
- :math:`t \in [0,1]` = parameter along line segment

Algorithm Parameters
++++++++++++++++++++

Key parameters affecting performance:

- **Number of particles** (n_particles): More particles = better exploration but slower
- **Maximum iterations** (max_iter): More iterations = better convergence but slower  
- **Inertia weight** (w): High = exploration, Low = exploitation
- **Cognitive coefficient** (c1): Attraction to personal best
- **Social coefficient** (c2): Attraction to global best

Typical values:
- n_particles: 20-50
- max_iter: 100-300
- w: 0.9 → 0.4 (linearly decreasing)
- c1, c2: 1.5-2.0

Advantages
++++++++++

- **Global optimization**: Can escape local minima unlike gradient-based methods
- **No derivatives needed**: Works with non-differentiable fitness landscapes
- **Parallel exploration**: Multiple particles search simultaneously
- **Simple implementation**: Few parameters and straightforward logic
- **Flexible**: Easily adaptable to different environments and constraints

Disadvantages
+++++++++++++

- **Stochastic**: Results may vary between runs
- **Parameter sensitive**: Performance heavily depends on parameter tuning
- **No optimality guarantee**: Metaheuristic without convergence proof
- **Computational cost**: Requires many fitness evaluations
- **Prone to stagnation**: Premature convergence where the entire swarm can get trapped in a local minimum if exploration is insufficient

Code Link
+++++++++

.. autofunction:: PathPlanning.ParticleSwarmOptimization.particle_swarm_optimization.main

Usage Example
+++++++++++++

.. code-block:: python

   import matplotlib.pyplot as plt
   from PathPlanning.ParticleSwarmOptimization.particle_swarm_optimization import main
   
   # Run PSO path planning with visualization
   main()

References
++++++++++

- `Particle swarm optimization - Wikipedia <https://en.wikipedia.org/wiki/Particle_swarm_optimization>`__
- Kennedy, J.; Eberhart, R. (1995). "Particle Swarm Optimization". Proceedings of IEEE International Conference on Neural Networks. IV. pp. 1942–1948.
- Shi, Y.; Eberhart, R. (1998). "A Modified Particle Swarm Optimizer". IEEE International Conference on Evolutionary Computation.
- `A Gentle Introduction to Particle Swarm Optimization <https://machinelearningmastery.com/a-gentle-introduction-to-particle-swarm-optimization/>`__
- Clerc, M.; Kennedy, J. (2002). "The particle swarm - explosion, stability, and convergence in a multidimensional complex space". IEEE Transactions on Evolutionary Computation. 6 (1): 58–73.