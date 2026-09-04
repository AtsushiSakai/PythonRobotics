GPS and IMU Fusion
------------------

This example estimates a planar robot pose from a body-frame inertial
measurement unit (IMU) and intermittent global-positioning-system (GPS)
measurements.  The state contains position, velocity, yaw, accelerometer
bias, and gyroscope bias:

.. math::

   \mathbf{x} = [x, y, v_x, v_y, \psi, b_{a_x}, b_{a_y}, b_g]^T.

At each IMU sample, the measured acceleration is rotated into the world frame
after subtracting the estimated accelerometer bias.  Constant-acceleration
integration gives

.. math::

   \mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k\Delta t
      + \frac{1}{2}R(\psi_k)(\mathbf{a}_k-\mathbf{b}_{a,k})\Delta t^2,

.. math::

   \mathbf{v}_{k+1} = \mathbf{v}_k
      + R(\psi_k)(\mathbf{a}_k-\mathbf{b}_{a,k})\Delta t,

.. math::

   \psi_{k+1} = \operatorname{wrap}(\psi_k + (\omega_k-b_{g,k})\Delta t).

The bias states follow random walks.  A GPS position update uses

.. math::

   \mathbf{z}_k = [x_k, y_k]^T + \mathbf{v}_k.

The implementation uses the Joseph covariance update to preserve symmetry and
offers an optional Mahalanobis gate for rejecting isolated GPS outliers.

Code Link
~~~~~~~~~

.. autoclass:: Localization.gps_imu_fusion.gps_imu_fusion.GPSIMUEKF
   :members:

.. autofunction:: Localization.gps_imu_fusion.gps_imu_fusion.run_simulation
