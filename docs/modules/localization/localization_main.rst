.. _localization:

Localization
============

.. include:: extended_kalman_filter_localization.rst


Unscented Kalman Filter localization
------------------------------------

|2|

This is a sensor fusion localization with Unscented Kalman Filter(UKF).

The lines and points are same meaning of the EKF simulation.

References:
~~~~~~~~~~~

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

How to calculate covariance matrix from particles
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The covariance matrix :math:`\Xi` from particle information is calculated by the following equation:

.. math:: \Xi_{j,k}=\frac{1}{1-\sum^N_{i=1}(w^i)^2}\sum^N_{i=1}w^i(x^i_j-\mu_j)(x^i_k-\mu_k)

- :math:`\Xi_{j,k}` is covariance matrix element at row :math:`i` and column :math:`k`.

- :math:`w^i` is the weight of the :math:`i` th particle.

- :math:`x^i_j` is the :math:`j` th state of the :math:`i` th particle.

- :math:`\mu_j` is the :math:`j` th mean state of particles.

References:
~~~~~~~~~~~

-  `PROBABILISTIC ROBOTICS`_
-  `Improving the particle filter in high dimensions using conjugate artificial process noise`_

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

References:
~~~~~~~~~~~

-  `PROBABILISTIC ROBOTICS`_

.. _PROBABILISTIC ROBOTICS: http://www.probabilistic-robotics.org/
.. _Discriminatively Trained Unscented Kalman Filter for Mobile Robot Localization: https://www.researchgate.net/publication/267963417_Discriminatively_Trained_Unscented_Kalman_Filter_for_Mobile_Robot_Localization
.. _Improving the particle filter in high dimensions using conjugate artificial process noise: https://arxiv.org/pdf/1801.07000.pdf

.. |2| image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/unscented_kalman_filter/animation.gif
.. |3| image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/particle_filter/animation.gif
.. |4| image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/histogram_filter/animation.gif
