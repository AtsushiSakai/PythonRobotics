Object shape recognition using circle fitting
---------------------------------------------

This is an object shape recognition using circle fitting.

Each range observation supplies a pair of coordinates. The two input lists
must have equal lengths so each point contributes once to the fit.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/circle_fitting/animation.gif

The blue circle is the true object shape.

The red crosses are observations from a ranging sensor.

The red circle is the estimated object shape using circle fitting.

Code Link
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. autofunction:: Mapping.circle_fitting.circle_fitting.circle_fitting
