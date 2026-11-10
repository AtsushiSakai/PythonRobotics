FastSLAM 2.0
------------

This is a feature based SLAM example using FastSLAM 2.0.

The animation has the same meanings as one of FastSLAM 1.0.

Resampling copies each selected pose and landmark estimate from the original
particle set. Landmark positions and covariances remain independent between
offspring, including particles selected from the same parent.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/SLAM/FastSLAM2/animation.gif

Code Link
~~~~~~~~~~~

.. autofunction:: SLAM.FastSLAM2.fast_slam2.fast_slam2


References
~~~~~~~~~~

- `PROBABILISTIC ROBOTICS <http://www.probabilistic-robotics.org/>`_

- `SLAM simulations by Tim Bailey <http://www-personal.acfr.usyd.edu.au/tbailey/software/slam_simulations.htm>`_

