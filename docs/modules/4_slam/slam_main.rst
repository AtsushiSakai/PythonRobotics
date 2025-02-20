.. _`SLAM`:

SLAM
====

Simultaneous Localization and Mapping(SLAM) examples
Simultaneous Localization and Mapping (SLAM) is an ability to estimate the pose of a robot and the map of the environment at the same time. The SLAM problem is hard to
solve, because a map is needed for localization and localization is needed for mapping. In this way, SLAM is often said to be similar to a ‘chicken-and-egg’ problem. Popular SLAM solution methods include the extended Kalman filter, particle filter, and Fast SLAM algorithm[31]. Fig.4 shows SLAM simulation results using extended Kalman filter and results using FastSLAM2.0[31].

.. toctree::
   :maxdepth: 2
   :caption: Contents

   iterative_closest_point_matching/iterative_closest_point_matching
   ekf_slam/ekf_slam
   FastSLAM1/FastSLAM1
   FastSLAM2/FastSLAM2
   graph_slam/graph_slam
