Object shape recognition using rectangle fitting
------------------------------------------------

This is an object shape recognition using rectangle fitting.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/rectangle_fitting/animation.gif

This example code is based on this paper algorithm:

- `Efficient L\-Shape Fitting for Vehicle Detection Using Laser Scanners \- The Robotics Institute Carnegie Mellon University <https://www.ri.cmu.edu/publications/efficient-l-shape-fitting-for-vehicle-detection-using-laser-scanners>`_

The algorithm consists of 2 steps as below.

Step1: Adaptive range segmentation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In the first step, all range data points are segmented into some clusters.

We calculate the distance between each range data and the nearest range data, and if this distance is below a certain threshold, it is judged to be in the same cluster. 

This distance threshold is determined in proportion to the distance from the sensor. 
This is taking advantage of the general model of distance sensors, which tends to have sparser data distribution as the distance from the sensor increases.

The threshold range is calculated by:

.. math:: r_{th} = R_0 + R_d * r_{origin}

where

- :math:`r_{th}`: Threashold range
- :math:`R_0, R_d`: Constant parameters
- :math:`r_{origin}`: Distance from the sensor for a range data.

Step2: Rectangle search
~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Area criteria
==================

2. Closeness criteria
======================

3. Variance criteria
=======================


References
~~~~~~~~~~

- `Efficient L\-Shape Fitting for Vehicle Detection Using Laser Scanners \- The Robotics Institute Carnegie Mellon University <https://www.ri.cmu.edu/publications/efficient-l-shape-fitting-for-vehicle-detection-using-laser-scanners>`_
