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

