GPS/IMU Fusion Localization with Bias Estimation
================================================

This example uses an extended Kalman filter (EKF) to combine body-frame
accelerometer and gyroscope measurements with GPS positions in a local metric
frame. IMU prediction runs at 20 Hz and GPS correction at 1 Hz. The simulation
compares fusion with and without bias estimation and IMU-only dead reckoning
along a figure-eight trajectory, including a GPS outage from 20 to 30 seconds.

.. image:: https://raw.githubusercontent.com/AtsushiSakai/PythonRoboticsGifs/5d1bb293772b94b19e7795d4265891017b5925f6/Localization/gps_imu_fusion/animation.gif
   :alt: GPS and IMU fusion localization with path comparisons, position errors, and accelerometer and gyroscope bias estimates against ground truth

The lower three panels show the EKF's estimated accelerometer x/y biases
(m/s²) and gyroscope bias (deg/s) in blue, with ground truth as black dashed
lines. The true biases are 0.04 m/s², -0.03 m/s² and 0.4 deg/s, respectively.
Gray shading marks the GPS outage in the error and bias plots.

Assumptions
-----------

Motion is restricted to a horizontal plane. The IMU is aligned with the body
frame and its acceleration has already been compensated for gravity. GPS
positions are expressed in metres, not latitude and longitude. Sensors are
time-aligned and co-located. Initial heading and velocity are approximately
known. Roll, pitch, altitude, magnetometers and barometers are not modeled.

The example estimates constant or slowly varying accelerometer and gyroscope
biases. GPS observes position only, so heading and bias estimation depend on
motion and the initial uncertainty; they cannot all be recovered from a
stationary position measurement. No real sensor data, outlier rejection or
hardware-specific calibration is included.

State and inertial prediction
-----------------------------

The eight-dimensional state contains world position, world velocity, heading,
two body-frame accelerometer biases and one yaw-rate bias:

.. math::

   \mathbf{x} = [p_x, p_y, v_x, v_y, \psi, b_{ax}, b_{ay}, b_\omega]^T.

Let :math:`\mathbf{a}_m` and :math:`\omega_m` be the measured acceleration
and yaw rate. With the body-to-world rotation

.. math::

   R(\psi)=\begin{bmatrix}\cos\psi&-\sin\psi\\\sin\psi&\cos\psi\end{bmatrix},
   \qquad \mathbf{a}_w=R(\psi)(\mathbf{a}_m-\mathbf{b}_a),

the discrete prediction holds world acceleration constant over one sample:

.. math::

   \mathbf{p}^{-} &= \mathbf{p}+\mathbf{v}\Delta t+\tfrac12\mathbf{a}_w\Delta t^2,\\
   \mathbf{v}^{-} &= \mathbf{v}+\mathbf{a}_w\Delta t,\\
   \psi^{-} &= \operatorname{wrap}(\psi+(\omega_m-b_\omega)\Delta t),\\
   \mathbf{b}^{-} &= \mathbf{b}.

Heading is wrapped to :math:`[-\pi,\pi)`. Define
:math:`\mathbf{j}=[-a_{wy},a_{wx}]^T`. In block order
:math:`(\mathbf{p},\mathbf{v},\psi,\mathbf{b}_a,b_\omega)`, the Jacobians are

.. math::

   F=\begin{bmatrix}
   I&\Delta t I&\tfrac12\Delta t^2\mathbf{j}&-\tfrac12\Delta t^2R&0\\
   0&I&\Delta t\mathbf{j}&-\Delta t R&0\\
   0&0&1&0&-\Delta t\\
   0&0&0&I&0\\
   0&0&0&0&1
   \end{bmatrix},\qquad
   G=\begin{bmatrix}
   \tfrac12\Delta t^2R&0\\ \Delta t R&0\\ 0&\Delta t\\ 0&0\\ 0&0
   \end{bmatrix}.

Covariance propagates as

.. math::

   P^{-}=FPF^T+GQ_{\mathrm{imu}}G^T+Q_b.

:math:`Q_{\mathrm{imu}}` is the covariance of a discrete IMU sample.
For bias random-walk standard deviations specified per square root second,
the bias block of :math:`Q_b` is
:math:`\operatorname{diag}(\sigma_{bax}^2,\sigma_{bay}^2,\sigma_{b\omega}^2)\Delta t`;
its other entries are zero. The simulation adds independent Gaussian sensor
noise and fixed biases, while the filter allows those biases to vary slowly.

GPS correction and outages
--------------------------

GPS supplies :math:`\mathbf{z}=[p_x,p_y]^T` with measurement matrix
:math:`H=[I_2\;0_{2\times6}]` and covariance
:math:`R_{\mathrm{gps}}=\sigma_{\mathrm{gps}}^2 I_2`.

.. math::

   S &= HP^{-}H^T+R_{\mathrm{gps}},\\
   K &= P^{-}H^T S^{-1},\\
   \mathbf{x}^{+} &= \mathbf{x}^{-}+K(\mathbf{z}-H\mathbf{x}^{-}),\\
   P^{+} &= (I-KH)P^{-}(I-KH)^T+KR_{\mathrm{gps}}K^T.

The gain is computed with a linear solve, and the Joseph covariance update
helps preserve numerical symmetry and positive semidefiniteness. Between GPS
fixes, and throughout the outage, only IMU prediction is performed. Uncertainty
can grow during the outage and decreases when GPS corrections resume.

Comparison without bias estimation
----------------------------------

The baseline EKF estimates only :math:`[p_x,p_y,v_x,v_y,\psi]^T`. It assumes
zero accelerometer and gyroscope bias, so it uses the raw IMU measurements
without bias compensation. Its prediction uses the first five rows and
columns of :math:`F` and the first five rows of :math:`G`, with no bias
random-walk term. GPS still corrects position, velocity and heading through
their cross-covariances. Thus it differs from IMU-only dead reckoning, which
never receives GPS corrections.

All three methods use exactly the same IMU samples, including the same nonzero
sensor biases and noise. Both EKFs receive the same GPS fixes and outage
schedule, and start with the same pose, velocity and corresponding covariance.
Only the eight-state EKF estimates and compensates for the biases. The red
dashed curves show the five-state EKF without bias estimation.

The analytic reference trajectory is independent of the filter's discrete
integrator. A fixed random seed makes the example reproducible, and the error
plot shows each method's Euclidean position error. These synthetic results
are not a real-sensor accuracy claim.

Code
----

.. autofunction:: Localization.gps_imu_fusion.gps_imu_fusion.predict

.. autofunction:: Localization.gps_imu_fusion.gps_imu_fusion.update_gps

.. autofunction:: Localization.gps_imu_fusion.gps_imu_fusion.simulate

References
----------

- `Oliver J. Woodman, An introduction to inertial navigation, University of Cambridge, 2007 <https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf>`_
- :doc:`../extended_kalman_filter_localization_files/extended_kalman_filter_localization`
