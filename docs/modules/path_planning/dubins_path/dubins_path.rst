Dubins path planning
--------------------

A sample code for Dubins path planning.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DubinsPath/animation.gif?raw=True

Dubins path
~~~~~~~~~~~~
Dubins path is a analytical path planning algorithm for a simple car model.

It can generates a shortest path between 2D poses (x, y, yaw) with maximum curvture comstraint and tangent(yaw angle) constraint.

The path consist of 3 segments of maximum curvature curves or a straight line segment.

Each segment type can is categorized by 3 type: 'Right turn (R)' , 'Left turn (L)', and 'Straight (S).' 

Possible path will be at least one of these six types: RSR, RSL, LSR, LSL, RLR, LRL. 

For example, one of RSR Dubins paths is:

.. image:: dubins_path/RSR.jpg
   :width: 400px

one of RLR Dubins paths is:

.. image:: dubins_path/RLR.jpg
   :width: 200px

Dubins path planner can output three types and distances of each course segment.

You can generate a path from these information and the maximum curvature information.

In the example code, a path which is minimum course length one among 6 course type is selected and then a path is constructed.

Code
~~~~~~~~~~~~~~~~~~~~

.. automodule:: PathPlanning.DubinsPath.dubins_path_planner
    :members:


Reference
~~~~~~~~~~~~~~~~~~~~

-  `Dubins path - Wikipedia <https://en.wikipedia.org/wiki/Dubins_path>`__
-  `15.3.1 Dubins Curves <http://planning.cs.uiuc.edu/node821.html>`__
-  `A Comprehensive, Step-by-Step Tutorial to Computing Dubin’s Paths <https://gieseanw.wordpress.com/2012/10/21/a-comprehensive-step-by-step-tutorial-to-computing-dubins-paths/>`__
