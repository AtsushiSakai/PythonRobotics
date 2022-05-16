Dubins path planning
--------------------

A sample code for Dubins path planning.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DubinsPath/animation.gif?raw=True

Dubins path
~~~~~~~~~~~~
Dubins path is a analytical path planning algorithm for a simple car model.

It can generates a shortest path between 2D poses (x, y, yaw) with maximum curvature constraint and tangent(yaw angle) constraint.

The path consist of 3 segments of maximum curvature curves or a straight line segment.

Each segment type can is categorized by 3 type: 'Right turn (R)' , 'Left turn (L)', and 'Straight (S).' 

Possible path will be at least one of these six types: RSR, RSL, LSR, LSL, RLR, LRL. 

For example, a RSR Dubins path is:

.. image:: RSR.jpg
   :width: 400px

:math:`p^2 = 2 + d ^ 2 - 2cos_{ab} + 2d(sin_a - sin_b)`

:math:`t = atan((cos_b - cos_a), d + sin_a - sin_b)`

:math:`d1 = mod(-\alpha + t)`

:math:`d2 = p`

:math:`d3 = mod(\beta - t)`

a RLR Dubins path is:

.. image:: RLR.jpg
   :width: 200px

Dubins path planner can output three types and distances of each course segment.

You can generate a path from these information and the maximum curvature information.

A path type which has minimum course length one among 6 types is selected,
and then a path is constructed based on the selected type.

API
~~~~~~~~~~~~~~~~~~~~

.. autofunction:: PathPlanning.DubinsPath.dubins_path_planner.plan_dubins_path


Reference
~~~~~~~~~~~~~~~~~~~~
-  `On Curves of Minimal Length with a Constraint on Average Curvature, and with Prescribed Initial and Terminal Positions and Tangents <https://www.jstor.org/stable/2372560?origin=crossref>`__
-  `Dubins path - Wikipedia <https://en.wikipedia.org/wiki/Dubins_path>`__
-  `15.3.1 Dubins Curves <http://planning.cs.uiuc.edu/node821.html>`__
-  `A Comprehensive, Step-by-Step Tutorial to Computing Dubin’s Paths <https://gieseanw.wordpress.com/2012/10/21/a-comprehensive-step-by-step-tutorial-to-computing-dubins-paths/>`__
