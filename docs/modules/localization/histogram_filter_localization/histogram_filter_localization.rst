Histogram filter localization
-----------------------------

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/histogram_filter/animation.gif

This is a 2D localization example with Histogram filter.

The red cross is true position, black points are RFID positions.

The blue grid shows a position probability of histogram filter.

In this simulation, we assume the robot's yaw orientation and RFID's positions are known,
but x,y positions are unknown.

The filter integrates speed input and range observations from RFID to known
localization.

Initial position information is not needed.

Filtering algorithm
~~~~~~~~~~~~~~~~~~~~

Histogram filter is a discrete Bayes filter in continuous space.

It uses regular girds to manage probability of the robot existence.

If a grid has higher probability, it means that the robot is likely to be there.

In the simulation, we want to estimate x-y position, so we use 2D grid data.

There are 4 steps for the histogram filter to estimate the probability as below:

Step1: Filter initialization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Histogram filter does not need initial position information.

In that case, we can initialize each grid probability as a same value.

If we can use initial position information, we can set initial probabilities based on it.

:ref:`gaussian_grid_map` might be useful when the initial position information is provided as gaussian distribution.

Step2: Predict probability by motion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In histogram filter, when a robot move to a next grid,
all probability information of each grid are shifted towards the movement direction.

This process represents the change in the probability distribution as the robot moves.

After the robot has moved, the probability distribution needs reflect
the estimation error due to the movement.

The `gaussian filter <https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.gaussian_filter.html>`_
is used in the simulation for adding noize.

Step3: Update probability by observation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Step4: Estimate position from probability
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


References:
~~~~~~~~~~~

-  `PROBABILISTIC ROBOTICS`_
