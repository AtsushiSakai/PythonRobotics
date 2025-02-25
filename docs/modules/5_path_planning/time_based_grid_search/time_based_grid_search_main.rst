Time based grid search
----------------------

Space-time astar
~~~~~~~~~~~~~~~~~~~~~~

This is an extension of A* algorithm that supports planning around dynamic obstacles.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/TimeBasedPathPlanning/SpaceTimeAStar/path_animation.gif

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/TimeBasedPathPlanning/SpaceTimeAStar/path_animation2.gif

The key difference of this algorithm compared to vanilla A* is that the cost and heuristic are now time-based instead of distance-based.
Using a time-based cost and heuristic ensures the path found is optimal in terms of time to reach the goal.

The cost is the amount of time it takes to reach a given node, and the heuristic is the minimum amount of time it could take to reach the goal from that node, disregarding all obstacles.
For a simple scenario where the robot can move 1 cell per time step and stop and go as it pleases, the heuristic for time is equivalent to the heuristic for distance.

References:
~~~~~~~~~~~

-  `Cooperative Pathfinding <https://www.davidsilver.uk/wp-content/uploads/2020/03/coop-path-AIWisdom.pdf>`__
