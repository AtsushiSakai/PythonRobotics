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

   Q = [(s, [s])], \qquad s \in M

where each element of \(Q\) is a pair \((v, P)\) with a current node \(v\) and
its path history \(P\).

The BFS expansion step (pseudocode):

.. math::

   \begin{aligned}
   (r,c),\;P &= Q.pop(0), \\
   	ext{for each } (r',c') \in N(r,c):\quad &\text{if } 0\le r'<R,\ 0\le c'<C,\ M_{r',c'}=0,\ (r',c')\notin\text{visited},\\
  &\quad \text{then } \text{visited} \leftarrow \text{visited} \cup \{(r',c')\},\\
  &\quad Q.append\big((r',c'),\; P + [(r',c')]\big).
  \end{aligned}

The algorithm halts when the target node \(t\) is reached. Because BFS explores
nodes in order of increasing distance, it returns a shortest path (by move count)
for static grids.


Dynamic Components
------------------


### Moving Target

Every few frames, the target moves randomly to an adjacent open cell:

.. math::

   T_{t+1} = T_t + \Delta_t,\qquad \Delta_t \in \{(-1,0),(1,0),(0,-1),(0,1)\}

with the constraint that the new position must be inside the grid and on a free cell.

This simulates dynamic goals or moving entities in robotic navigation.

### Evolving Obstacles

With a small probability :math:`p`, each cell toggles between *free* and *blocked*:

.. math::

   M_{i,j}^{t+1} = \begin{cases}
      1 - M_{i,j}^{t}, & \text{with probability } p,\\
      M_{i,j}^{t}, & \text{with probability } 1-p.
   \end{cases}

This reflects real-world conditions like temporary obstructions or environment changes.


Visualization
-------------

The maze, solver, target, and BFS layers are visualized using **Matplotlib**.

Elements include:

- **Maze cells** - magma colormap (black = wall, bright = open)
- **Visited nodes** - blue overlay with transparency
- **Path line** - green connecting line
- **Solver (robot)** - cyan circle
- **Target** - magenta star
- **Breadcrumbs** - trail of previously visited solver positions

A sample animation frame:

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/dynamic_maze_solver/animation.gif
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

   L_t = |P_t|, \qquad E_t = |V_t|

where \(P_t\) is the discovered path at frame \(t\) and \(V_t\) is the set of visited nodes.

Remarks:

- BFS returns a shortest-path in terms of number of grid moves when the grid is static.
- When the environment changes over time, the solver must recompute; this makes optimality
  relative to the latest observed configuration rather than the original static grid.


Code Link
++++++++

.. automodule:: PathPlanning.BreadthFirstSearch.dynamic_maze_solver


References
----------

- **Algorithm:** Breadth-First Search (BFS) :-`<https://en.wikipedia.org/wiki/Breadth-first_search>`_
- **Visualization:** Matplotlib animation
- **Maze Solver:**:-`<https://medium.com/@luthfisauqi17_68455/artificial-intelligence-search-problem-solve-maze-using-breadth-first-search-bfs-algorithm-255139c6e1a3>`__



