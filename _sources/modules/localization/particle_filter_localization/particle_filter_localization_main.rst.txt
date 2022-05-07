Particle filter localization
----------------------------

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/particle_filter/animation.gif

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

- `_PROBABILISTIC ROBOTICS: <http://www.probabilistic-robotics.org>`_
- `Improving the particle filter in high dimensions using conjugate artificial process noise <https://arxiv.org/pdf/1801.07000.pdf>`_
