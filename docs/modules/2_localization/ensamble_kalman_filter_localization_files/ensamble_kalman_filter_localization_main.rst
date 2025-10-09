Ensemble Kalman Filter Localization
-----------------------------------

.. figure:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/ensamble_kalman_filter/animation.gif

This is a sensor fusion localization with Ensemble Kalman Filter(EnKF).

The blue line is true trajectory, the black line is dead reckoning trajectory,
and the red line is estimated trajectory with EnKF.

The red ellipse is estimated covariance ellipse with EnKF.

It is assumed that the robot can measure distance and bearing angle from landmarks (RFID).

These measurements are used for EnKF localization.

Code Link
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. autofunction:: Localization.ensemble_kalman_filter.ensemble_kalman_filter.enkf_localization


Ensemble Kalman Filter Algorithm
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Ensemble Kalman Filter (EnKF) is a Monte Carlo approach to the Kalman filter that
uses an ensemble of state estimates to represent the probability distribution. Unlike
the traditional Kalman filter that propagates the mean and covariance analytically,
EnKF uses a collection of samples (ensemble members) to estimate the state and its uncertainty.

The EnKF algorithm consists of two main steps:

=== Predict ===

For each ensemble member :math:`i = 1, ..., N`:

1. Add random noise to the control input:

   :math:`\mathbf{u}^i = \mathbf{u} + \epsilon_u`, where :math:`\epsilon_u \sim \mathcal{N}(0, \mathbf{R})`

2. Predict the state:

   :math:`\mathbf{x}^i_{pred} = f(\mathbf{x}^i_t, \mathbf{u}^i)`

3. Predict the observation:

   :math:`\mathbf{z}^i_{pred} = h(\mathbf{x}^i_{pred}) + \epsilon_z`, where :math:`\epsilon_z \sim \mathcal{N}(0, \mathbf{Q})`

=== Update ===

1. Calculate ensemble mean for states:

   :math:`\bar{\mathbf{x}} = \frac{1}{N}\sum_{i=1}^N \mathbf{x}^i_{pred}`

2. Calculate ensemble mean for observations:

   :math:`\bar{\mathbf{z}} = \frac{1}{N}\sum_{i=1}^N \mathbf{z}^i_{pred}`

3. Calculate state deviation:

   :math:`\mathbf{X}' = \mathbf{x}^i_{pred} - \bar{\mathbf{x}}`

4. Calculate observation deviation:

   :math:`\mathbf{Z}' = \mathbf{z}^i_{pred} - \bar{\mathbf{z}}`

5. Calculate covariance matrices:

   :math:`\mathbf{U} = \frac{1}{N-1}\mathbf{X}'\mathbf{Z}'^T`

   :math:`\mathbf{V} = \frac{1}{N-1}\mathbf{Z}'\mathbf{Z}'^T`

6. Calculate Kalman gain:

   :math:`\mathbf{K} = \mathbf{U}\mathbf{V}^{-1}`

7. Update each ensemble member:

   :math:`\mathbf{x}^i_{t+1} = \mathbf{x}^i_{pred} + \mathbf{K}(\mathbf{z}_{obs} - \mathbf{z}^i_{pred})`

8. Calculate final state estimate:

   :math:`\mathbf{x}_{est} = \frac{1}{N}\sum_{i=1}^N \mathbf{x}^i_{t+1}`

9. Calculate covariance estimate:

   :math:`\mathbf{P}_{est} = \frac{1}{N}\sum_{i=1}^N (\mathbf{x}^i_{t+1} - \mathbf{x}_{est})(\mathbf{x}^i_{t+1} - \mathbf{x}_{est})^T`


Filter Design
~~~~~~~~~~~~~

In this simulation, the robot has a state vector with 4 states at time :math:`t`:

.. math:: \mathbf{x}_t = [x_t, y_t, \phi_t, v_t]^T

where:

- :math:`x, y` are 2D position coordinates
- :math:`\phi` is orientation (yaw angle)
- :math:`v` is velocity

The filter uses an ensemble of :math:`N=20` particles to represent the state distribution.

Input Vector
^^^^^^^^^^^^

The robot has a velocity sensor and a gyro sensor, so the control input vector at each time step is:

.. math:: \mathbf{u}_t = [v_t, \omega_t]^T

where:

- :math:`v_t` is linear velocity
- :math:`\omega_t` is angular velocity (yaw rate)

The input vector includes sensor noise modeled by covariance matrix :math:`\mathbf{R}`:

.. math:: \mathbf{R} = \begin{bmatrix} \sigma_v^2 & 0 \\ 0 & \sigma_\omega^2 \end{bmatrix}


Observation Vector
^^^^^^^^^^^^^^^^^^

The robot can observe landmarks (RFID tags) with distance and bearing measurements:

.. math:: \mathbf{z}_t = [d_t, \theta_t, x_{lm}, y_{lm}]

where:

- :math:`d_t` is the distance to the landmark
- :math:`\theta_t` is the bearing angle to the landmark
- :math:`x_{lm}, y_{lm}` are the known landmark positions

The observation noise is modeled by covariance matrix :math:`\mathbf{Q}`:

.. math:: \mathbf{Q} = \begin{bmatrix} \sigma_d^2 & 0 \\ 0 & \sigma_\theta^2 \end{bmatrix}


Motion Model
~~~~~~~~~~~~

The robot motion model is:

.. math::  \dot{x} = v \cos(\phi)

.. math::  \dot{y} = v \sin(\phi)

.. math::  \dot{\phi} = \omega

.. math::  \dot{v} = 0

Discretized with time step :math:`\Delta t`, the motion model becomes:

.. math:: \mathbf{x}_{t+1} = f(\mathbf{x}_t, \mathbf{u}_t) = \mathbf{F}\mathbf{x}_t + \mathbf{B}\mathbf{u}_t

where:

:math:`\begin{equation*} \mathbf{F} = \begin{bmatrix} 1 & 0 & 0 & 0\\ 0 & 1 & 0 & 0\\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 \\ \end{bmatrix} \end{equation*}`

:math:`\begin{equation*} \mathbf{B} = \begin{bmatrix} \cos(\phi) \Delta t & 0\\ \sin(\phi) \Delta t & 0\\ 0 & \Delta t\\ 1 & 0\\ \end{bmatrix} \end{equation*}`

The motion function expands to:

:math:`\begin{equation*} \begin{bmatrix} x_{t+1} \\ y_{t+1} \\ \phi_{t+1} \\ v_{t+1} \end{bmatrix} = \begin{bmatrix} x_t + v_t\cos(\phi_t)\Delta t \\ y_t + v_t\sin(\phi_t)\Delta t \\ \phi_t + \omega_t \Delta t \\ v_t \end{bmatrix} \end{equation*}`


Observation Model
~~~~~~~~~~~~~~~~~

For each landmark at position :math:`(x_{lm}, y_{lm})`, the observation model calculates
the expected landmark position in the global frame:

.. math:: 
   \begin{bmatrix} 
   x_{lm,obs} \\ 
   y_{lm,obs} 
   \end{bmatrix} = 
   \begin{bmatrix} 
   x + d \cos(\phi + \theta) \\ 
   y + d \sin(\phi + \theta) 
   \end{bmatrix}

where :math:`d` and :math:`\theta` are the observed distance and bearing to the landmark.

The observation function projects the robot state to expected landmark positions,
which are then compared with actual landmark positions for the update step.


Advantages of EnKF
~~~~~~~~~~~~~~~~~~

- **No Jacobian required**: Unlike EKF, EnKF does not need to compute Jacobian matrices, making it easier to implement for nonlinear systems
- **Handles non-Gaussian distributions**: The ensemble representation can capture non-Gaussian features of the state distribution
- **Computationally efficient**: For high-dimensional systems, EnKF can be more efficient than maintaining full covariance matrices
- **Easy to parallelize**: Each ensemble member can be propagated independently


Reference
~~~~~~~~~

- `Ensemble Kalman filtering <https://rmets.onlinelibrary.wiley.com/doi/10.1256/qj.05.135>`_
- `PROBABILISTIC ROBOTICS <http://www.probabilistic-robotics.org/>`_

