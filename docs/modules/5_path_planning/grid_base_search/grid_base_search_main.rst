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

