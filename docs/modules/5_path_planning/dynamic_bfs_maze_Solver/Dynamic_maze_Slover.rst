Dynamic Maze Solver using Breadth-First Search (BFS)
====================================================

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

This example demonstrates a **dynamic maze-solving algorithm** based on the
**Breadth-First Search (BFS)** strategy. The visualizer dynamically updates a maze
in real-time while the solver attempts to reach a moving target. 

Unlike static pathfinding examples, this version introduces:

- **A moving target** that relocates periodically.
- **Randomly evolving obstacles** that can appear or disappear.
- **Animated BFS exploration**, showing visited cells, computed paths, and breadcrumbs.

This simulation provides intuition for dynamic pathfinding problems such as
robot navigation in unpredictable environments.


Algorithmic Background
----------------------

### Breadth-First Search (BFS)

The BFS algorithm is a graph traversal method that explores nodes in layers, 
guaranteeing the shortest path in an unweighted grid.

Let the maze be represented as a grid:

.. math::

   M = \{ (i, j) \mid 0 \leq i < R, 0 \leq j < C \}

where each cell is either *free (0)* or *obstacle (1)*.

The BFS frontier expands as:

.. math::

   Q = [(s, [s])]

where *s* is the start position, and the second term is the path history.

At each iteration:

.. math::

   (r, c), path = Q.pop(0)

   \text{for each neighbor } (r', c') \text{ in } N(r, c):
       \text{if } (r', c') \text{ is free and unvisited:}
           Q.append((r', c'), path + [(r', c')])

The algorithm halts when the target node *t* is reached.

Because BFS explores all nodes in increasing distance order, the path returned
is the shortest (in terms of number of moves).


Dynamic Components
------------------

### Moving Target

Every few frames, the target moves randomly to an adjacent open cell:

.. math::

   T_{new} = T_{old} + \Delta

where :math:`\Delta \in \{ (-1,0), (1,0), (0,-1), (0,1) \}`.

This simulates dynamic goals or moving entities in robotic navigation.

### Evolving Obstacles

With a small probability :math:`p`, each cell toggles between *free* and *blocked*:

.. math::

   M_{i,j}^{t+1} =
   \begin{cases}
       1 - M_{i,j}^{t} & \text{with probability } p \\
       M_{i,j}^{t} & \text{otherwise}
   \end{cases}

This reflects real-world conditions like temporary obstructions or environment changes.


Visualization
-------------

The maze, solver, target, and BFS layers are visualized using **Matplotlib**.

Elements include:

- **Maze cells** – magma colormap (black = wall, bright = open)
- **Visited nodes** – blue overlay with transparency
- **Path line** – green connecting line
- **Solver (robot)** – cyan circle
- **Target** – magenta star
- **Breadcrumbs** – trail of previously visited solver positions

A sample animation frame:

.. image:: ezgif.com-crop.jpg
   :alt: Maze BFS dynamic visualizer frame
   :align: center
   :scale: 80 %


Mathematical Insights
---------------------

- **BFS guarantees optimality** in unweighted grids.
- The evolving maze introduces **non-stationarity**, requiring recomputation per frame.
- The path length :math:`L_t` fluctuates as the environment changes.

If :math:`E_t` is the set of explored nodes at frame :math:`t`, then:

.. math::

   L_t = |P_t|, \quad E_t = |V_t|

where :math:`P_t` is the discovered path and :math:`V_t` is the visited node set.

The solver continually re-estimates the path to accommodate new maze configurations.


References
----------

- **Algorithm:** Breadth-First Search (BFS) :-`<https://en.wikipedia.org/wiki/Breadth-first_search>`_
- **Visualization:** Matplotlib animation
- **Maze Solver:**:-`<https://medium.com/@luthfisauqi17_68455/artificial-intelligence-search-problem-solve-maze-using-breadth-first-search-bfs-algorithm-255139c6e1a3>`__



