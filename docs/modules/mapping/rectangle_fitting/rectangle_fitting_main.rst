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

In the second step, for each cluster calculated in the previous step, rectangular fittings will be applied.
In this rectangular fitting, each cluster's distance data is rotated at certain angle intervals.
It is evaluated by one of the three evaluation functions below, then best angle parameter one is selected as the rectangle shape.

1. Rectangle Area Minimization criteria
=========================================

This evaluation function calculates the area of the smallest rectangle that includes all the points, derived from the difference between the maximum and minimum values on the x-y axis for all distance data points. 
This allows for fitting a rectangle in a direction that encompasses as much of the smallest rectangular shape as possible.


2. Closeness criteria
======================

This evaluation function uses the distances between the top and bottom vertices on the right side of the rectangle and each point in the distance data as evaluation values. 
If there are points on the rectangle edges, this evaluation value decreases.

3. Variance criteria
=======================

This evaluation function uses the squreed distances between the edges of the rectangle (horizontal and vertical) and each point. 
Calculating the squared error is the same as calculating the variance.
The smaller this variance, the more it signifies that the points fit within the rectangle.

API
~~~~~~

.. autoclass:: Mapping.rectangle_fitting.rectangle_fitting.LShapeFitting
	:members:

References
~~~~~~~~~~

- `Efficient L\-Shape Fitting for Vehicle Detection Using Laser Scanners \- The Robotics Institute Carnegie Mellon University <https://www.ri.cmu.edu/publications/efficient-l-shape-fitting-for-vehicle-detection-using-laser-scanners>`_
