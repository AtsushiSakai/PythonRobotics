
Extended Kalman Filter Localization
-----------------------------------

.. figure:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/Localization/extended_kalman_filter/animation.gif
   :alt: EKF

   EKF

This is a sensor fusion localization with Extended Kalman Filter(EKF).

The blue line is true trajectory, the black line is dead reckoning
trajectory,

the green point is positioning observation (ex. GPS), and the red line
is estimated trajectory with EKF.

The red ellipse is estimated covariance ellipse with EKF.

Code; `PythonRobotics/extended_kalman_filter.py at master ·
AtsushiSakai/PythonRobotics <https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/extended_kalman_filter/extended_kalman_filter.py>`__

Kalman Filter
~~~~~~~~~~~~~

Ref
~~~

-  `PROBABILISTIC-ROBOTICS.ORG <http://www.probabilistic-robotics.org/>`__
