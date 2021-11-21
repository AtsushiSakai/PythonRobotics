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


.. include:: ekf_slam.rst


.. include:: FastSLAM1.rst

FastSLAM 2.0
------------

This is a feature based SLAM example using FastSLAM 2.0.

The animation has the same meanings as one of FastSLAM 1.0.

|6|

References
~~~~~~~~~~

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

.. include:: graphSLAM_doc.rst
.. include:: graphSLAM_formulation.rst
.. include:: graphSLAM_SE2_example.rst

Ref:

-  `A Tutorial on Graph-Based SLAM`_

.. _`Introduction to Mobile Robotics: Iterative Closest Point Algorithm`: https://cs.gmu.edu/~kosecka/cs685/cs685-icp.pdf
.. _PROBABILISTIC ROBOTICS: http://www.probabilistic-robotics.org/
.. _SLAM simulations by Tim Bailey: http://www-personal.acfr.usyd.edu.au/tbailey/software/slam_simulations.htm
.. _A Tutorial on Graph-Based SLAM: http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf
.. _FastSLAM Lecture: http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam10-fastslam.pdf

.. [blanco2010tutorial] Blanco, J.-L.A tutorial onSE(3) transformation parameterization and on-manifold optimization.University of Malaga, Tech. Rep 3(2010)
.. [grisetti2010tutorial] Grisetti, G., Kummerle, R., Stachniss, C., and Burgard, W.A tutorial on graph-based SLAM.IEEE Intelligent Transportation Systems Magazine 2, 4 (2010), 31–43.

.. |3| image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/SLAM/iterative_closest_point/animation.gif
.. |5| image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/SLAM/FastSLAM1/animation.gif
.. |6| image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/SLAM/FastSLAM2/animation.gif
.. |7| image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/SLAM/GraphBasedSLAM/animation.gif
