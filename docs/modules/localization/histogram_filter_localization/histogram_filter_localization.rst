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

Histogram filter

Step1: Filter initialization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Step1: Update probability by motion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Step2: Add uncertainty to probability
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Step3: Update probability by observation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Step4: Estimate position from probability
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


References:
~~~~~~~~~~~

-  `PROBABILISTIC ROBOTICS`_
