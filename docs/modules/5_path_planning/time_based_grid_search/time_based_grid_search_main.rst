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

Code Link
^^^^^^^^^^^^^
.. autoclass:: PathPlanning.TimeBasedPathPlanning.SpaceTimeAStar.SpaceTimeAStar


Safe Interval Path Planning
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The safe interval path planning algorithm is described in this paper:

`SIPP: Safe Interval Path Planning for Dynamic Environments <https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf>`__

It is faster than space-time A* because it pre-computes the intervals of time that are unoccupied in each cell. This allows it to reduce the number of successor node it generates by avoiding nodes within the same interval.

**Comparison with SpaceTime A*:**

Arrangement 1 starting at (1, 18)

SafeInterval planner::

    Found path to goal after 322 expansions
    Planning took: 0.00730 seconds

SpaceTime A*::

    Found path to goal after 2717154 expansions
    Planning took: 20.51330 seconds

**Benchmarking the Safe Interval Path Planner:**

250 random obstacles::

    Found path to goal after 764 expansions
    Planning took: 0.60596 seconds

.. image:: https://raw.githubusercontent.com/AtsushiSakai/PythonRoboticsGifs/refs/heads/master/PathPlanning/TimeBasedPathPlanning/SafeIntervalPathPlanner/path_animation.gif

Arrangement 1 starting at (1, 18)::

    Found path to goal after 322 expansions
    Planning took: 0.00730 seconds

.. image:: https://raw.githubusercontent.com/AtsushiSakai/PythonRoboticsGifs/refs/heads/master/PathPlanning/TimeBasedPathPlanning/SafeIntervalPathPlanner/path_animation2.gif

Code Link
^^^^^^^^^^^^^
.. autoclass:: PathPlanning.TimeBasedPathPlanning.SafeInterval.SafeIntervalPathPlanner

Multi-Agent Path Planning
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Priority Based Planning
~~~~~~~~~~~~~~~~~~~~

TODO: explain the algorithm

TODO: explain why spacetime a* is so much slower

Using SpaceTimeAstar for the single agent planner::
    Using planner: <class 'PathPlanning.TimeBasedPathPlanning.SpaceTimeAStar.SpaceTimeAStar'>

    Planning for agent:  StartAndGoal(index=1, start=Position(x=1, y=1), goal=Position(x=19, y=18))
    Found path to goal after 300 expansions

    Planning for agent:  StartAndGoal(index=2, start=Position(x=1, y=2), goal=Position(x=19, y=17))
    Found path to goal after 645 expansions

    Planning for agent:  StartAndGoal(index=3, start=Position(x=1, y=3), goal=Position(x=19, y=16))
    Found path to goal after 273 expansions

    Planning for agent:  StartAndGoal(index=4, start=Position(x=1, y=4), goal=Position(x=19, y=15))
    Found path to goal after 7798 expansions

    Planning for agent:  StartAndGoal(index=5, start=Position(x=1, y=5), goal=Position(x=19, y=14))
    Found path to goal after 186 expansions

    Planning for agent:  StartAndGoal(index=6, start=Position(x=1, y=6), goal=Position(x=19, y=13))
    Found path to goal after 216790 expansions

    Planning for agent:  StartAndGoal(index=7, start=Position(x=1, y=7), goal=Position(x=19, y=12))
    Found path to goal after 472128 expansions

    Planning for agent:  StartAndGoal(index=8, start=Position(x=1, y=8), goal=Position(x=19, y=11))
    Found path to goal after 916485 expansions

    Planning for agent:  StartAndGoal(index=9, start=Position(x=1, y=9), goal=Position(x=19, y=10))
    Found path to goal after 2098980 expansions

    Planning for agent:  StartAndGoal(index=10, start=Position(x=1, y=10), goal=Position(x=19, y=9))
    Found path to goal after 50 expansions

    Planning for agent:  StartAndGoal(index=11, start=Position(x=1, y=11), goal=Position(x=19, y=8))
    Found path to goal after 62 expansions

    Planning for agent:  StartAndGoal(index=12, start=Position(x=1, y=12), goal=Position(x=19, y=7))
    ...
    (timed out)

Using the safe interval path planner::

    Planning for agent:  StartAndGoal(index=1, start=Position(x=1, y=1), goal=Position(x=19, y=18))
    Found path to goal after 256 expansions

    Planning for agent:  StartAndGoal(index=2, start=Position(x=1, y=2), goal=Position(x=19, y=17))
    Found path to goal after 336 expansions

    Planning for agent:  StartAndGoal(index=3, start=Position(x=1, y=3), goal=Position(x=19, y=16))
    Found path to goal after 526 expansions

    Planning for agent:  StartAndGoal(index=4, start=Position(x=1, y=4), goal=Position(x=19, y=15))
    Found path to goal after 619 expansions

    Planning for agent:  StartAndGoal(index=5, start=Position(x=1, y=5), goal=Position(x=19, y=14))
    Found path to goal after 534 expansions

    Planning for agent:  StartAndGoal(index=6, start=Position(x=1, y=6), goal=Position(x=19, y=13))
    Found path to goal after 596 expansions

    Planning for agent:  StartAndGoal(index=7, start=Position(x=1, y=7), goal=Position(x=19, y=12))
    Found path to goal after 149 expansions

    Planning for agent:  StartAndGoal(index=8, start=Position(x=1, y=8), goal=Position(x=19, y=11))
    Found path to goal after 551 expansions

    Planning for agent:  StartAndGoal(index=9, start=Position(x=1, y=9), goal=Position(x=19, y=10))
    Found path to goal after 92 expansions

    Planning for agent:  StartAndGoal(index=10, start=Position(x=1, y=10), goal=Position(x=19, y=9))
    Found path to goal after 1042 expansions

    Planning for agent:  StartAndGoal(index=11, start=Position(x=1, y=11), goal=Position(x=19, y=8))
    Found path to goal after 1062 expansions

    Planning for agent:  StartAndGoal(index=12, start=Position(x=1, y=12), goal=Position(x=19, y=7))
    Found path to goal after 1000 expansions

    Planning for agent:  StartAndGoal(index=13, start=Position(x=1, y=13), goal=Position(x=19, y=6))
    Found path to goal after 867 expansions

    Planning for agent:  StartAndGoal(index=14, start=Position(x=1, y=14), goal=Position(x=19, y=5))
    Found path to goal after 964 expansions

    Planning for agent:  StartAndGoal(index=15, start=Position(x=1, y=15), goal=Position(x=19, y=4))
    Found path to goal after 1146 expansions

    Planning took: 0.11520 seconds


References
~~~~~~~~~~~

-  `Cooperative Pathfinding <https://www.davidsilver.uk/wp-content/uploads/2020/03/coop-path-AIWisdom.pdf>`__
-  `SIPP: Safe Interval Path Planning for Dynamic Environments <https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf>`__
