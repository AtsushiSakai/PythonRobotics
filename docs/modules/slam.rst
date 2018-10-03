.. _slam:

SLAM
====

Simultaneous Localization and Mapping(SLAM) examples

.. _iterative-closest-point-(icp)-matching:

Iterative Closest Point (ICP) Matching
--------------------------------------

This is a 2D ICP matching example with singular value decomposition.

It can calculate a rotation matrix and a translation vector between
points to points.

|3|

Ref:

-  `Introduction to Mobile Robotics: Iterative Closest Point Algorithm`_

EKF SLAM
--------

This is an Extended Kalman Filter based SLAM example.

The blue line is ground truth, the black line is dead reckoning, the red
line is the estimated trajectory with EKF SLAM.

The green crosses are estimated landmarks.

|4|

Ref:

-  `PROBABILISTIC ROBOTICS`_

FastSLAM 1.0
------------

This is a feature based SLAM example using FastSLAM 1.0.

The blue line is ground truth, the black line is dead reckoning, the red
line is the estimated trajectory with FastSLAM.

The red points are particles of FastSLAM.

Black points are landmarks, blue crosses are estimated landmark
positions by FastSLAM.

|5|

Ref:

-  `PROBABILISTIC ROBOTICS`_

-  `SLAM simulations by Tim Bailey`_

FastSLAM 2.0
------------

This is a feature based SLAM example using FastSLAM 2.0.

The animation has the same meanings as one of FastSLAM 1.0.

|6|

Ref:

-  `PROBABILISTIC ROBOTICS`_

-  `SLAM simulations by Tim Bailey`_

Graph based SLAM
----------------

This is a graph based SLAM example.

The blue line is ground truth.

The black line is dead reckoning.

The red line is the estimated trajectory with Graph based SLAM.

The black stars are landmarks for graph edge generation.

|7|

Ref:

-  `A Tutorial on Graph-Based SLAM`_

.. _`Introduction to Mobile Robotics: Iterative Closest Point Algorithm`: https://cs.gmu.edu/~kosecka/cs685/cs685-icp.pdf
.. _PROBABILISTIC ROBOTICS: http://www.probabilistic-robotics.org/
.. _SLAM simulations by Tim Bailey: http://www-personal.acfr.usyd.edu.au/tbailey/software/slam_simulations.htm
.. _A Tutorial on Graph-Based SLAM: http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf

.. |3| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/SLAM/iterative_closest_point/animation.gif
.. |4| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/SLAM/EKFSLAM/animation.gif
.. |5| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/SLAM/FastSLAM1/animation.gif
.. |6| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/SLAM/FastSLAM2/animation.gif
.. |7| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/SLAM/GraphBasedSLAM/animation.gif
