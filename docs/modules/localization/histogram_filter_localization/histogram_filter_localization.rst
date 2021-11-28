Histogram filter localization
-----------------------------

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/histogram_filter/animation.gif

This is a 2D localization example with Histogram filter.

The red cross is true position, black points are RFID positions.

The blue grid shows a position probability of histogram filter.

In this simulation, x,y are unknown, yaw is known.

The filter integrates speed input and range observations from RFID for
localization.

Initial position is not needed.

References:
~~~~~~~~~~~

-  `PROBABILISTIC ROBOTICS`_
