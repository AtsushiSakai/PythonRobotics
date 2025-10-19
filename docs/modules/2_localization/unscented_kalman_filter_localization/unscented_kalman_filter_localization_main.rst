Unscented Kalman Filter localization
------------------------------------

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/unscented_kalman_filter/animation.gif

This is a sensor fusion localization with Unscented Kalman Filter(UKF).

The blue line is true trajectory, the black line is dead reckoning trajectory,
the green points are GPS observations, and the red line is estimated trajectory with UKF.

The red ellipse is estimated covariance ellipse with UKF.

Code Link
~~~~~~~~~~~~~

.. autofunction:: Localization.unscented_kalman_filter.unscented_kalman_filter.ukf_estimation


Unscented Kalman Filter Algorithm
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Unscented Kalman Filter (UKF) is a nonlinear state estimation technique that uses
the unscented transform to handle nonlinearities. Unlike the Extended Kalman Filter (EKF)
that linearizes the nonlinear functions using Jacobians, UKF uses a deterministic sampling
approach called sigma points to capture the mean and covariance of the state distribution.

The UKF algorithm consists of two main steps:

=== Predict ===

1. Generate sigma points around the current state estimate:

   :math:`\chi_0 = \mathbf{x}_{t}`

   :math:`\chi_i = \mathbf{x}_{t} + \gamma\sqrt{\mathbf{P}_t}_i` for :math:`i = 1, ..., n`

   :math:`\chi_i = \mathbf{x}_{t} - \gamma\sqrt{\mathbf{P}_t}_{i-n}` for :math:`i = n+1, ..., 2n`

   where :math:`\gamma = \sqrt{n + \lambda}` and :math:`\lambda = \alpha^2(n + \kappa) - n`

2. Propagate sigma points through the motion model:

   :math:`\chi^-_i = f(\chi_i, \mathbf{u}_t)`

3. Calculate predicted state mean:

   :math:`\mathbf{x}^-_{t+1} = \sum_{i=0}^{2n} w^m_i \chi^-_i`

4. Calculate predicted state covariance:

   :math:`\mathbf{P}^-_{t+1} = \sum_{i=0}^{2n} w^c_i (\chi^-_i - \mathbf{x}^-_{t+1})(\chi^-_i - \mathbf{x}^-_{t+1})^T + \mathbf{Q}`

=== Update ===

1. Generate sigma points around the predicted state:

   :math:`\chi_i = \text{generate\_sigma\_points}(\mathbf{x}^-_{t+1}, \mathbf{P}^-_{t+1})`

2. Propagate sigma points through the observation model:

   :math:`\mathcal{Z}_i = h(\chi_i)`

3. Calculate predicted observation mean:

   :math:`\mathbf{z}^-_{t+1} = \sum_{i=0}^{2n} w^m_i \mathcal{Z}_i`

4. Calculate innovation covariance:

   :math:`\mathbf{S}_t = \sum_{i=0}^{2n} w^c_i (\mathcal{Z}_i - \mathbf{z}^-_{t+1})(\mathcal{Z}_i - \mathbf{z}^-_{t+1})^T + \mathbf{R}`

5. Calculate cross-correlation matrix:

   :math:`\mathbf{P}_{xz} = \sum_{i=0}^{2n} w^c_i (\chi_i - \mathbf{x}^-_{t+1})(\mathcal{Z}_i - \mathbf{z}^-_{t+1})^T`

6. Calculate Kalman gain:

   :math:`\mathbf{K} = \mathbf{P}_{xz}\mathbf{S}_t^{-1}`

7. Update state estimate:

   :math:`\mathbf{x}_{t+1} = \mathbf{x}^-_{t+1} + \mathbf{K}(\mathbf{z}_t - \mathbf{z}^-_{t+1})`

8. Update covariance estimate:

   :math:`\mathbf{P}_{t+1} = \mathbf{P}^-_{t+1} - \mathbf{K}\mathbf{S}_t\mathbf{K}^T`


Sigma Points and Weights
~~~~~~~~~~~~~~~~~~~~~~~~~

The UKF uses deterministic sigma points to represent the state distribution. The weights
for combining sigma points are calculated as:

**Mean weights:**

:math:`w^m_0 = \frac{\lambda}{n + \lambda}`

:math:`w^m_i = \frac{1}{2(n + \lambda)}` for :math:`i = 1, ..., 2n`

**Covariance weights:**

:math:`w^c_0 = \frac{\lambda}{n + \lambda} + (1 - \alpha^2 + \beta)`

:math:`w^c_i = \frac{1}{2(n + \lambda)}` for :math:`i = 1, ..., 2n`

where:

- :math:`\alpha` controls the spread of sigma points around the mean (typically :math:`0.001 \leq \alpha \leq 1`)
- :math:`\beta` incorporates prior knowledge of the distribution (:math:`\beta = 2` is optimal for Gaussian distributions)
- :math:`\kappa` is a secondary scaling parameter (typically :math:`\kappa = 0`)
- :math:`n` is the dimension of the state vector


Filter Design
~~~~~~~~~~~~~

In this simulation, the robot has a state vector with 4 states at time :math:`t`:

.. math:: \mathbf{x}_t = [x_t, y_t, \phi_t, v_t]^T

where:

- :math:`x, y` are 2D position coordinates
- :math:`\phi` is orientation (yaw angle)
- :math:`v` is velocity

In the code, "xEst" means the state vector.

The covariance matrices are:

- :math:`\mathbf{P}_t` is the state covariance matrix
- :math:`\mathbf{Q}` is the process noise covariance matrix
- :math:`\mathbf{R}` is the observation noise covariance matrix

**Process Noise Covariance** :math:`\mathbf{Q}`:

.. math:: 
   \mathbf{Q} = \begin{bmatrix} 
   0.1^2 & 0 & 0 & 0 \\
   0 & 0.1^2 & 0 & 0 \\
   0 & 0 & (0.017)^2 & 0 \\
   0 & 0 & 0 & 1.0^2
   \end{bmatrix}

**Observation Noise Covariance** :math:`\mathbf{R}`:

.. math:: 
   \mathbf{R} = \begin{bmatrix} 
   1.0^2 & 0 \\
   0 & 1.0^2
   \end{bmatrix}


Input Vector
^^^^^^^^^^^^

The robot has a velocity sensor and a gyro sensor, so the control input vector at each time step is:

.. math:: \mathbf{u}_t = [v_t, \omega_t]^T

where:

- :math:`v_t` is linear velocity [m/s]
- :math:`\omega_t` is angular velocity (yaw rate) [rad/s]

The input vector includes sensor noise.


Observation Vector
^^^^^^^^^^^^^^^^^^

The robot has a GNSS (GPS) sensor that can measure x-y position:

.. math:: \mathbf{z}_t = [x_t, y_t]^T

The observation includes GPS noise with covariance :math:`\mathbf{R}`.


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

:math:`\Delta t = 0.1` [s] is the time interval.


Observation Model
~~~~~~~~~~~~~~~~~

The robot can get x-y position information from GPS.

The GPS observation model is:

.. math:: \mathbf{z}_{t} = h(\mathbf{x}_t) = \mathbf{H} \mathbf{x}_t

where:

:math:`\begin{equation*} \mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ \end{bmatrix} \end{equation*}`

The observation function states that:

:math:`\begin{equation*} \begin{bmatrix} x_{obs} \\ y_{obs} \end{bmatrix} = h(\mathbf{x}) = \begin{bmatrix} x \\ y \end{bmatrix} \end{equation*}`


UKF Parameters
~~~~~~~~~~~~~~

The UKF uses three tuning parameters:

- **ALPHA = 0.001**: Controls the spread of sigma points around the mean. Smaller values result in sigma points closer to the mean.
- **BETA = 2**: Incorporates prior knowledge about the distribution. For Gaussian distributions, the optimal value is 2.
- **KAPPA = 0**: Secondary scaling parameter. Usually set to 0 or 3-n where n is the state dimension.

These parameters affect the calculation of :math:`\lambda = \alpha^2(n + \kappa) - n`, which determines
the sigma point spread and weights.


Advantages of UKF over EKF
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Unscented Kalman Filter offers several advantages over the Extended Kalman Filter:

- **No Jacobian required**: UKF does not need to compute Jacobian matrices, which can be error-prone and computationally expensive for complex nonlinear systems
- **Higher accuracy**: UKF captures the mean and covariance to the second order (third order for Gaussian distributions) while EKF only achieves first-order accuracy
- **Better handling of nonlinearities**: The unscented transform provides a better approximation of the probability distribution after nonlinear transformation
- **Easier implementation**: For highly nonlinear systems, UKF is often easier to implement since it doesn't require analytical derivatives
- **Numerical stability**: UKF can be more numerically stable than EKF for strongly nonlinear systems


Reference
~~~~~~~~~~~

- `Discriminatively Trained Unscented Kalman Filter for Mobile Robot Localization <https://www.researchgate.net/publication/267963417_Discriminatively_Trained_Unscented_Kalman_Filter_for_Mobile_Robot_Localization>`_
- `The Unscented Kalman Filter for Nonlinear Estimation <https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf>`_
- `PROBABILISTIC ROBOTICS <http://www.probabilistic-robotics.org/>`_
