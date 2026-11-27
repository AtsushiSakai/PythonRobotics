Grid based search
-----------------

Breadth First Search
~~~~~~~~~~~~~~~~~~~~

This is a 2D grid based path planning with Breadth first search algorithm.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/BreadthFirstSearch/animation.gif

In the animation, cyan points are searched nodes.

Code Link
+++++++++++++

.. autofunction:: PathPlanning.BreadthFirstSearch.breadth_first_search.BreadthFirstSearchPlanner


Depth First Search
~~~~~~~~~~~~~~~~~~~~

This is a 2D grid based path planning with Depth first search algorithm.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DepthFirstSearch/animation.gif

In the animation, cyan points are searched nodes.

Code Link
+++++++++++++

.. autofunction:: PathPlanning.DepthFirstSearch.depth_first_search.DepthFirstSearchPlanner


.. _dijkstra:

Dijkstra algorithm
~~~~~~~~~~~~~~~~~~

This is a 2D grid based shortest path planning with Dijkstra's algorithm.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/Dijkstra/animation.gif

In the animation, cyan points are searched nodes.

Code Link
+++++++++++++

.. autofunction:: PathPlanning.Dijkstra.dijkstra.DijkstraPlanner


.. _a*-algorithm:

A\* algorithm
~~~~~~~~~~~~~

This is a 2D grid based shortest path planning with A star algorithm.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/AStar/animation.gif

In the animation, cyan points are searched nodes.

Its heuristic is 2D Euclid distance.

Code Link
+++++++++++++

.. autofunction:: PathPlanning.AStar.a_star.AStarPlanner


.. _jump-point-search-algorithm:

Jump Point Search algorithm
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This is a 2D grid based shortest path planning with the Jump Point Search (JPS)
algorithm.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/JumpPointSearch/animation.gif

In the animation, cyan points are the searched jump points.

Jump Point Search is an optimization of the A star algorithm for uniform-cost
grid maps. On such a grid, many paths with the same length are just reorderings
of the same moves, and ordinary A star wastes time expanding all of them. JPS
removes this symmetry with two ideas:

- **Neighbor pruning.** When a node is reached from a given direction, most of
  its neighbors can only be reached optimally through the node's predecessor,
  so they are discarded. Only the *natural* neighbors (the straight-ahead
  continuation, and for a diagonal move its two component directions) are kept.

- **Forced neighbors.** An obstacle beside the current node can block the
  alternative shortest path that would otherwise bypass the node. The neighbor
  on the far side of that obstacle then becomes *forced*: it must be reached
  through the current node, so it is kept.

Instead of stepping one cell at a time, JPS "jumps" along a direction until it
reaches the goal, a node with a forced neighbor, or (for a diagonal jump) a node
from which a horizontal or vertical jump finds such a node. These stopping cells
are the *jump points*, and only they are placed on the open list. The search
therefore expands far fewer nodes than A star while returning a path with the
**same optimal cost**, because it explores exactly the same eight-connected grid
(diagonal moves cost :math:`\sqrt{2}` and are allowed whenever the target cell is
free, as in the A star example above).

Its heuristic is the 2D Euclidean distance, the same as the A star example.

Reference
++++++++++++

-  `Online Graph Pruning for Pathfinding on Grid Maps (Harabor and Grastien, AAAI 2011) <https://ojs.aaai.org/index.php/AAAI/article/view/7994>`__
-  `Jump point search - Wikipedia <https://en.wikipedia.org/wiki/Jump_point_search>`__

Code Link
+++++++++++++

.. autofunction:: PathPlanning.JumpPointSearch.jump_point_search.JumpPointSearchPlanner


Bidirectional A\* algorithm
~~~~~~~~~~~~~~~~~~~~~~~~~~~

This is a 2D grid based shortest path planning with bidirectional A star algorithm.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/BidirectionalAStar/animation.gif

In the animation, cyan points are searched nodes.

Code Link
+++++++++++++

.. autofunction:: PathPlanning.BidirectionalAStar.bidirectional_a_star.BidirectionalAStarPlanner


.. _Theta*-algorithm:

Theta\* algorithm
~~~~~~~~~~~~~~~~~

This is a 2D grid based shortest path planning with Theta star algorithm.

It offers an optimization over the A star algorithm to generate any-angle paths. Unlike A star, which always assigns the current node as the parent of the successor, Theta star checks for a line-of-sight (unblocked path) from an earlier node (typically the parent of the current node) to the successor, and directly assigns it as a parent if visible. This reduces cost by replacing staggered segments with straight lines. 

Checking for line of sight involves verifying that no obstacles block the straight line between two nodes. On a grid, this means identifying all the discrete cells that the line passes through to determine if they are empty. Bresenham’s line algorithm is commonly used for this purpose. Starting from one endpoint, it incrementally steps along one axis, while considering the gradient to determine the position on the other axis. Because it relies only on integer addition and subtraction, it is both efficient and precise for grid-based visibility checks in Theta star.

As a result, Theta star produces shorter, smoother paths than A star, ideal for ground or aerial robots operating in continuous environments where smoother motion enables higher acceleration and reduced travel time.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ThetaStar/animation.gif

In the animation, each cyan arrow represents a node pointing to its parent.

Reference
++++++++++++

-  `Theta*: Any-Angle Path Planning on Grids <https://cdn.aaai.org/AAAI/2007/AAAI07-187.pdf>`__
-  `Bresenham's line algorithm <https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm>`__

Code Link
+++++++++++++

.. autofunction:: PathPlanning.ThetaStar.theta_star.ThetaStarPlanner


.. _D*-algorithm:

D\* algorithm
~~~~~~~~~~~~~

This is a 2D grid based shortest path planning with D star algorithm.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DStar/animation.gif

The animation shows a robot finding its path avoiding an obstacle using the D* search algorithm.

Code Link
+++++++++++++

.. autoclass:: PathPlanning.DStar.dstar.Dstar


Reference
++++++++++++

-  `D* search Wikipedia <https://en.wikipedia.org/wiki/D*>`__

D\* lite algorithm
~~~~~~~~~~~~~~~~~~

This is a 2D grid based path planning and replanning with D star lite algorithm.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DStarLite/animation.gif

Code Link
+++++++++++++

.. autoclass:: PathPlanning.DStarLite.d_star_lite.DStarLite

Reference
++++++++++++

- `Improved Fast Replanning for Robot Navigation in Unknown Terrain <http://www.cs.cmu.edu/~maxim/files/dlite_icra02.pdf>`_


Potential Field algorithm
~~~~~~~~~~~~~~~~~~~~~~~~~

This is a 2D grid based path planning with Potential Field algorithm.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/PotentialFieldPlanning/animation.gif

In the animation, the blue heat map shows potential value on each grid.

Code Link
+++++++++++++

.. autofunction:: PathPlanning.PotentialFieldPlanning.potential_field_planning.potential_field_planning


Reference
++++++++++++

-  `Robotic Motion Planning:Potential
   Functions <https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf>`__

