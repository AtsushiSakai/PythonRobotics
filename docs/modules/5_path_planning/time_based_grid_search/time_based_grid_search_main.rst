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

One optimization that was added in `this PR <https://github.com/AtsushiSakai/PythonRobotics/pull/1183>`__ was to add an expanded set to the algorithm. The algorithm will not expand nodes that are already in that set. This greatly reduces the number of node expansions needed to find a path, since no duplicates are expanded. It also helps to reduce the amount of memory the algorithm uses.

Before::

    Found path to goal after 204490 expansions
    Planning took: 1.72464 seconds
    Memory usage (RSS): 68.19 MB


After::

    Found path to goal after 2348 expansions
    Planning took: 0.01550 seconds
    Memory usage (RSS): 64.85 MB

When starting at (1, 11) in the structured obstacle arrangement (second of the two gifs above).

References:
~~~~~~~~~~~

-  `Cooperative Pathfinding <https://www.davidsilver.uk/wp-content/uploads/2020/03/coop-path-AIWisdom.pdf>`__
