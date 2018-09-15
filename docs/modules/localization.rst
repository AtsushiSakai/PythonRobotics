.. _localization:

Localization
============

Extended Kalman Filter localization
-----------------------------------

.. raw:: html

   <img src="https://github.com/AtsushiSakai/PythonRobotics/raw/master/Localization/extended_kalman_filter/animation.gif" width="640">

This is a sensor fusion localization with Extended Kalman Filter(EKF).

The blue line is true trajectory, the black line is dead reckoning
trajectory,

the green point is positioning observation (ex. GPS), and the red line
is estimated trajectory with EKF.

The red ellipse is estimated covariance ellipse with EKF.

Ref:

-  `PROBABILISTIC ROBOTICS`_

Unscented Kalman Filter localization
------------------------------------

|2|

This is a sensor fusion localization with Unscented Kalman Filter(UKF).

The lines and points are same meaning of the EKF simulation.

Ref:

-  `Discriminatively Trained Unscented Kalman Filter for Mobile Robot
   Localization`_

Particle filter localization
----------------------------

|3|

This is a sensor fusion localization with Particle Filter(PF).

The blue line is true trajectory, the black line is dead reckoning
trajectory,

and the red line is estimated trajectory with PF.

It is assumed that the robot can measure a distance from landmarks
(RFID).

This measurements are used for PF localization.

Ref:

-  `PROBABILISTIC ROBOTICS`_

Histogram filter localization
-----------------------------

|4|

This is a 2D localization example with Histogram filter.

The red cross is true position, black points are RFID positions.

The blue grid shows a position probability of histogram filter.

In this simulation, x,y are unknown, yaw is known.

The filter integrates speed input and range observations from RFID for
localization.

Initial position is not needed.

Ref:

-  `PROBABILISTIC ROBOTICS`_

.. _PROBABILISTIC ROBOTICS: http://www.probabilistic-robotics.org/
.. _Discriminatively Trained Unscented Kalman Filter for Mobile Robot Localization: https://www.researchgate.net/publication/267963417_Discriminatively_Trained_Unscented_Kalman_Filter_for_Mobile_Robot_Localization

.. |2| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/Localization/unscented_kalman_filter/animation.gif
.. |3| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/Localization/particle_filter/animation.gif
.. |4| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/Localization/histogram_filter/animation.gif
