.. _path_planning:

Path Planning
=============

Dynamic Window Approach
-----------------------

This is a 2D navigation sample code with Dynamic Window Approach.

-  `The Dynamic Window Approach to Collision
   Avoidance <https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf>`__

|DWA|

Grid based search
-----------------

Dijkstra algorithm
~~~~~~~~~~~~~~~~~~

This is a 2D grid based shortest path planning with Dijkstra's
algorithm.

|Dijkstra|

In the animation, cyan points are searched nodes.

.. _a*-algorithm:

A\* algorithm
~~~~~~~~~~~~~

This is a 2D grid based shortest path planning with A star algorithm.

|astar|

In the animation, cyan points are searched nodes.

Its heuristic is 2D Euclid distance.

Potential Field algorithm
~~~~~~~~~~~~~~~~~~~~~~~~~

This is a 2D grid based path planning with Potential Field algorithm.

|PotentialField|

In the animation, the blue heat map shows potential value on each grid.

Ref:

-  `Robotic Motion Planning:Potential
   Functions <https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf>`__

Model Predictive Trajectory Generator
-------------------------------------

This is a path optimization sample on model predictive trajectory
generator.

This algorithm is used for state lattice planner.

Path optimization sample
~~~~~~~~~~~~~~~~~~~~~~~~

|4|

Lookup table generation sample
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

|5|

Ref:

-  `Optimal rough terrain trajectory generation for wheeled mobile
   robots <http://journals.sagepub.com/doi/pdf/10.1177/0278364906075328>`__

State Lattice Planning
----------------------

This script is a path planning code with state lattice planning.

This code uses the model predictive trajectory generator to solve
boundary problem.

Ref:

-  `Optimal rough terrain trajectory generation for wheeled mobile
   robots <http://journals.sagepub.com/doi/pdf/10.1177/0278364906075328>`__

-  `State Space Sampling of Feasible Motions for High-Performance Mobile
   Robot Navigation in Complex
   Environments <http://www.frc.ri.cmu.edu/~alonzo/pubs/papers/JFR_08_SS_Sampling.pdf>`__

Uniform polar sampling
~~~~~~~~~~~~~~~~~~~~~~

|6|

Biased polar sampling
~~~~~~~~~~~~~~~~~~~~~

|7|

Lane sampling
~~~~~~~~~~~~~

|8|

.. _probabilistic-road-map-(prm)-planning:

Probabilistic Road-Map (PRM) planning
-------------------------------------

|PRM|

This PRM planner uses Dijkstra method for graph search.

In the animation, blue points are sampled points,

Cyan crosses means searched points with Dijkstra method,

The red line is the final path of PRM.

Ref:

-  `Probabilistic roadmap -
   Wikipedia <https://en.wikipedia.org/wiki/Probabilistic_roadmap>`__

　　

Voronoi Road-Map planning
-------------------------

|VRM|

This Voronoi road-map planner uses Dijkstra method for graph search.

In the animation, blue points are Voronoi points,

Cyan crosses mean searched points with Dijkstra method,

The red line is the final path of Vornoi Road-Map.

Ref:

-  `Robotic Motion
   Planning <https://www.cs.cmu.edu/~motionplanning/lecture/Chap5-RoadMap-Methods_howie.pdf>`__

.. _rapidly-exploring-random-trees-(rrt):

Rapidly-Exploring Random Trees (RRT)
------------------------------------

Basic RRT
~~~~~~~~~

|9|

This is a simple path planning code with Rapidly-Exploring Random Trees
(RRT)

Black circles are obstacles, green line is a searched tree, red crosses
are start and goal positions.

.. _rrt*:

RRT\*
~~~~~

|10|

This is a path planning code with RRT\*

Black circles are obstacles, green line is a searched tree, red crosses
are start and goal positions.

Ref:

-  `Incremental Sampling-based Algorithms for Optimal Motion
   Planning <https://arxiv.org/abs/1005.0416>`__

-  `Sampling-based Algorithms for Optimal Motion
   Planning <http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.419.5503&rep=rep1&type=pdf>`__

RRT with dubins path
~~~~~~~~~~~~~~~~~~~~

|PythonRobotics|

Path planning for a car robot with RRT and dubins path planner.

.. _rrt*-with-dubins-path:

RRT\* with dubins path
~~~~~~~~~~~~~~~~~~~~~~

|AtsushiSakai/PythonRobotics|

Path planning for a car robot with RRT\* and dubins path planner.

.. _rrt*-with-reeds-sheep-path:

RRT\* with reeds-sheep path
~~~~~~~~~~~~~~~~~~~~~~~~~~~

|11|

Path planning for a car robot with RRT\* and reeds sheep path planner.

.. _informed-rrt*:

Informed RRT\*
~~~~~~~~~~~~~~

|irrt|

This is a path planning code with Informed RRT*.

The cyan ellipse is the heuristic sampling domain of Informed RRT*.

Ref:

-  `Informed RRT\*: Optimal Sampling-based Path Planning Focused via
   Direct Sampling of an Admissible Ellipsoidal
   Heuristic <https://arxiv.org/pdf/1404.2334.pdf>`__

.. _batch-informed-rrt*:

Batch Informed RRT\*
~~~~~~~~~~~~~~~~~~~~

|irrt2|

This is a path planning code with Batch Informed RRT*.

Ref:

-  `Batch Informed Trees (BIT*): Sampling-based Optimal Planning via the
   Heuristically Guided Search of Implicit Random Geometric
   Graphs <https://arxiv.org/abs/1405.5848>`__

.. _closed-loop-rrt*:

Closed Loop RRT\*
~~~~~~~~~~~~~~~~~

A vehicle model based path planning with closed loop RRT*.

|CLRRT|

In this code, pure-pursuit algorithm is used for steering control,

PID is used for speed control.

Ref:

-  `Motion Planning in Complex Environments using Closed-loop
   Prediction <http://acl.mit.edu/papers/KuwataGNC08.pdf>`__

-  `Real-time Motion Planning with Applications to Autonomous Urban
   Driving <http://acl.mit.edu/papers/KuwataTCST09.pdf>`__

-  `[1601.06326] Sampling-based Algorithms for Optimal Motion Planning
   Using Closed-loop Prediction <https://arxiv.org/abs/1601.06326>`__

.. _lqr-rrt*:

LQR-RRT\*
~~~~~~~~~

This is a path planning simulation with LQR-RRT*.

A double integrator motion model is used for LQR local planner.

|LQRRRT|

Ref:

-  `LQR-RRT\*: Optimal Sampling-Based Motion Planning with Automatically
   Derived Extension
   Heuristics <http://lis.csail.mit.edu/pubs/perez-icra12.pdf>`__

-  `MahanFathi/LQR-RRTstar: LQR-RRT\* method is used for random motion
   planning of a simple pendulum in its phase
   plot <https://github.com/MahanFathi/LQR-RRTstar>`__

Cubic spline planning
---------------------

A sample code for cubic path planning.

This code generates a curvature continuous path based on x-y waypoints
with cubic spline.

Heading angle of each point can be also calculated analytically.

|12|
|13|
|14|

B-Spline planning
-----------------

|B-Spline|

This is a path planning with B-Spline curse.

If you input waypoints, it generates a smooth path with B-Spline curve.

The final course should be on the first and last waypoints.

Ref:

-  `B-spline - Wikipedia <https://en.wikipedia.org/wiki/B-spline>`__

.. _eta^3-spline-path-planning:

Eta^3 Spline path planning
--------------------------

|eta3|

This is a path planning with Eta^3 spline.

Ref:

-  `\\eta^3-Splines for the Smooth Path Generation of Wheeled Mobile
   Robots <https://ieeexplore.ieee.org/document/4339545/>`__

Bezier path planning
--------------------

A sample code of Bezier path planning.

It is based on 4 control points Beier path.

|Bezier1|

If you change the offset distance from start and end point,

You can get different Beizer course:

|Bezier2|

Ref:

-  `Continuous Curvature Path Generation Based on Bezier Curves for
   Autonomous
   Vehicles <http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.294.6438&rep=rep1&type=pdf>`__

Quintic polynomials planning
----------------------------

Motion planning with quintic polynomials.

|2|

It can calculate 2D path, velocity, and acceleration profile based on
quintic polynomials.

Ref:

-  `Local Path Planning And Motion Control For Agv In
   Positioning <http://ieeexplore.ieee.org/document/637936/>`__

Dubins path planning
--------------------

A sample code for Dubins path planning.

|dubins|

Ref:

-  `Dubins path -
   Wikipedia <https://en.wikipedia.org/wiki/Dubins_path>`__

Reeds Shepp planning
--------------------

A sample code with Reeds Shepp path planning.

|RSPlanning|

Ref:

-  `15.3.2 Reeds-Shepp
   Curves <http://planning.cs.uiuc.edu/node822.html>`__

-  `optimal paths for a car that goes both forwards and
   backwards <https://pdfs.semanticscholar.org/932e/c495b1d0018fd59dee12a0bf74434fac7af4.pdf>`__

-  `ghliu/pyReedsShepp: Implementation of Reeds Shepp
   curve. <https://github.com/ghliu/pyReedsShepp>`__

LQR based path planning
-----------------------

A sample code using LQR based path planning for double integrator model.

|RSPlanning2|

Optimal Trajectory in a Frenet Frame
------------------------------------

|15|

This is optimal trajectory generation in a Frenet Frame.

The cyan line is the target course and black crosses are obstacles.

The red line is predicted path.

Ref:

-  `Optimal Trajectory Generation for Dynamic Street Scenarios in a
   Frenet
   Frame <https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf>`__

-  `Optimal trajectory generation for dynamic street scenarios in a
   Frenet Frame <https://www.youtube.com/watch?v=Cj6tAQe7UCY>`__

.. |DWA| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/DynamicWindowApproach/animation.gif
.. |Dijkstra| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/Dijkstra/animation.gif
.. |astar| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/AStar/animation.gif
.. |PotentialField| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/PotentialFieldPlanning/animation.gif
.. |4| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/ModelPredictiveTrajectoryGenerator/kn05animation.gif
.. |5| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/ModelPredictiveTrajectoryGenerator/lookuptable.png?raw=True
.. |6| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/StateLatticePlanner/UniformPolarSampling.gif
.. |7| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/StateLatticePlanner/BiasedPolarSampling.gif
.. |8| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/StateLatticePlanner/LaneSampling.gif
.. |PRM| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/ProbabilisticRoadMap/animation.gif
.. |VRM| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/VoronoiRoadMap/animation.gif
.. |9| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/RRT/animation.gif
.. |10| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/RRTstar/animation.gif
.. |PythonRobotics| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/RRTDubins/animation.gif
.. |AtsushiSakai/PythonRobotics| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/RRTStarDubins/animation.gif
.. |11| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/RRTStarReedsShepp/animation.gif
.. |irrt| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/InformedRRTStar/animation.gif
.. |irrt2| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/BatchInformedRRTStar/animation.gif
.. |CLRRT| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/ClosedLoopRRTStar/animation.gif
.. |LQRRRT| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/LQRRRTStar/animation.gif
.. |12| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/CubicSpline/Figure_1.png?raw=True
.. |13| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/CubicSpline/Figure_2.png?raw=True
.. |14| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/CubicSpline/Figure_3.png?raw=True
.. |B-Spline| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/BSplinePath/Figure_1.png?raw=True
.. |eta3| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/Eta3SplinePath/animation.gif?raw=True
.. |Bezier1| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/BezierPath/Figure_1.png?raw=True
.. |Bezier2| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/BezierPath/Figure_2.png?raw=True
.. |2| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/QuinticPolynomialsPlanner/animation.gif
.. |dubins| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/DubinsPath/animation.gif?raw=True
.. |RSPlanning| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/ReedsSheppPath/animation.gif?raw=true
.. |RSPlanning2| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/LQRPlanner/animation.gif?raw=true
.. |15| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathPlanning/FrenetOptimalTrajectory/animation.gif
