Particle Filter Localization
----------------------------

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/particle_filter/animation.gif

This is a sensor fusion localization with Particle Filter (PF), also known as Monte Carlo Localization (MCL).

The blue line is the true trajectory, the black line is the dead reckoning trajectory,
and the red line is the estimated trajectory with PF. The red dots represent individual particles,
and the red dashed ellipse shows the estimation uncertainty.

The robot measures distances to RFID landmarks, and these measurements are fused with
odometry data using the particle filter algorithm.

Overview
~~~~~~~~~~~~~

The Particle Filter is a non-parametric implementation of the Bayes filter that represents
the belief (probability distribution) of the robot's state using a set of weighted samples (particles).
Unlike parametric filters (e.g., Kalman Filter), it can represent arbitrary probability distributions,
including multi-modal distributions.

**Key advantages:**

- Can handle non-linear motion and observation models
- Can represent multi-modal distributions (e.g., ambiguous localization)
- No assumptions about Gaussian distributions
- Relatively simple to implement

**Key disadvantages:**

- Computationally expensive (scales with number of particles)
- Can suffer from particle degeneracy
- Requires many particles in high-dimensional state spaces

Code Link
~~~~~~~~~~~~~

.. autofunction:: Localization.particle_filter.particle_filter.pf_localization


Particle Filter Algorithm
~~~~~~~~~~~~~~~~~~~~~~~~~~

The particle filter algorithm consists of three main steps:

1. **Prediction Step (Motion Update)**
2. **Update Step (Measurement Update)**
3. **Resampling Step**

Algorithm Pseudocode
^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   For each particle i = 1 to N:
       1. Sample noisy control: u_i ~ p(u_t | u_t)
       2. Predict: x_i = f(x_{i,t-1}, u_i)
       3. For each observation z_j:
           4. w_i *= p(z_j | x_i)
   5. Normalize weights: w_i = w_i / sum(w_i)
   6. Estimate: x_est = sum(w_i * x_i)
   7. If N_eff < threshold:
       8. Resample particles

Mathematical Formulation
^^^^^^^^^^^^^^^^^^^^^^^^

**State Representation:**

The robot state at time :math:`t` is:

.. math:: \mathbf{x}_t = [x_t, y_t, \theta_t, v_t]^T

where :math:`x_t, y_t` is the 2D position, :math:`\theta_t` is the orientation (yaw), and :math:`v_t` is the velocity.

**Particle Set:**

The belief is represented by :math:`N` particles:

.. math:: \mathcal{X}_t = \{x_t^{[1]}, x_t^{[2]}, ..., x_t^{[N]}\}

with corresponding weights:

.. math:: \mathcal{W}_t = \{w_t^{[1]}, w_t^{[2]}, ..., w_t^{[N]}\}, \quad \sum_{i=1}^N w_t^{[i]} = 1


Motion Model
~~~~~~~~~~~~

The robot uses a simple kinematic model:

.. math::

   x_{t+1} = x_t + v_t \cos(\theta_t) \Delta t

.. math::

   y_{t+1} = y_t + v_t \sin(\theta_t) \Delta t

.. math::

   \theta_{t+1} = \theta_t + \omega_t \Delta t

.. math::

   v_{t+1} = v_t

where :math:`\mathbf{u}_t = [v_t, \omega_t]^T` is the control input (velocity and yaw rate).

In matrix form:

.. math:: \mathbf{x}_{t+1} = F\mathbf{x}_t + B\mathbf{u}_t

where

.. math::

   F = \begin{bmatrix}
   1 & 0 & 0 & 0\\
   0 & 1 & 0 & 0\\
   0 & 0 & 1 & 0 \\
   0 & 0 & 0 & 0
   \end{bmatrix}, \quad
   B = \begin{bmatrix}
   \cos(\theta_t) \Delta t & 0\\
   \sin(\theta_t) \Delta t & 0\\
   0 & \Delta t\\
   1 & 0
   \end{bmatrix}

**Process Noise:**

Control input noise is modeled as:

.. math:: \mathbf{u}_t \sim \mathcal{N}(\mathbf{u}_t, R)

where :math:`R = \text{diag}(\sigma_v^2, \sigma_\omega^2)` is the process noise covariance.


Observation Model
~~~~~~~~~~~~~~~~~

The robot observes ranges to RFID landmarks at known positions :math:`\mathbf{m}_j = [m_{j,x}, m_{j,y}]^T`.

The predicted observation is:

.. math:: \hat{z}_j = \sqrt{(x_t - m_{j,x})^2 + (y_t - m_{j,y})^2}

**Observation Noise:**

Range measurements have Gaussian noise:

.. math:: z_j \sim \mathcal{N}(\hat{z}_j, \sigma_z^2)

**Likelihood Function:**

The observation likelihood for particle :math:`i` is:

.. math:: p(z_j | x_t^{[i]}) = \frac{1}{\sqrt{2\pi\sigma_z^2}} \exp\left(-\frac{(z_j - \hat{z}_j^{[i]})^2}{2\sigma_z^2}\right)

For multiple observations, likelihoods are multiplied (assuming independence):

.. math:: w_t^{[i]} = w_{t-1}^{[i]} \prod_{j=1}^{M} p(z_j | x_t^{[i]})


Resampling
~~~~~~~~~~

**Particle Degeneracy Problem:**

Over time, most particles have negligible weights, leading to computational waste.
This is detected using the effective particle number:

.. math:: N_{eff} = \frac{1}{\sum_{i=1}^N (w_t^{[i]})^2}

When :math:`N_{eff} < N_{threshold}`, resampling is triggered.

**Low Variance Resampling (Systematic Resampling):**

This algorithm samples particles proportional to their weights with low variance:

1. Compute cumulative weight distribution: :math:`c^{[i]} = \sum_{j=1}^{i} w^{[j]}`
2. Generate systematic samples: :math:`r^{[i]} = \frac{i-1}{N} + \frac{U}{N}` where :math:`U \sim \text{Uniform}(0, 1/N)`
3. For each :math:`r^{[i]}`, find particle :math:`j` such that :math:`c^{[j-1]} < r^{[i]} \leq c^{[j]}`
4. Reset weights to uniform: :math:`w^{[i]} = 1/N`


How to Calculate Covariance Matrix from Particles
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The state estimate is the weighted mean:

.. math:: \hat{\mathbf{x}}_t = \sum_{i=1}^N w_t^{[i]} x_t^{[i]}

The covariance matrix :math:`\Xi` is calculated using weighted sample covariance with Bessel's correction:

.. math:: \Xi_{j,k}=\frac{1}{1-\sum^N_{i=1}(w^i)^2}\sum^N_{i=1}w^i(x^i_j-\mu_j)(x^i_k-\mu_k)

where:

- :math:`\Xi_{j,k}` is the covariance matrix element at row :math:`j` and column :math:`k`
- :math:`w^i` is the weight of the :math:`i`-th particle
- :math:`x^i_j` is the :math:`j`-th state component of the :math:`i`-th particle
- :math:`\mu_j` is the :math:`j`-th component of the mean state

The denominator :math:`(1-\sum^N_{i=1}(w^i)^2)^{-1}` is the correction factor for weighted samples,
which accounts for the effective sample size.


Comparison with Other Localization Methods
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Extended Kalman Filter (EKF)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Similarities:**

- Both use Bayes filter framework (predict and update)
- Both handle sensor fusion (odometry + observations)
- Both estimate mean and covariance

**Differences:**

+------------------------+--------------------------------+--------------------------------+
| Feature                | Particle Filter                | Extended Kalman Filter         |
+========================+================================+================================+
| Distribution           | Non-parametric (particles)     | Parametric (Gaussian)          |
+------------------------+--------------------------------+--------------------------------+
| Linearity              | Handles non-linear models      | Requires linearization         |
+------------------------+--------------------------------+--------------------------------+
| Multi-modal            | Yes (can represent multiple    | No (unimodal Gaussian only)    |
|                        | hypotheses)                    |                                |
+------------------------+--------------------------------+--------------------------------+
| Computational cost     | High (O(N) particles)          | Low (fixed, independent of     |
|                        |                                | uncertainty)                   |
+------------------------+--------------------------------+--------------------------------+
| Accuracy               | Converges to optimal with      | Optimal only for linear        |
|                        | infinite particles             | Gaussian systems               |
+------------------------+--------------------------------+--------------------------------+
| Initial uncertainty    | Can handle unknown initial     | Requires good initial          |
|                        | position (global localization) | estimate                       |
+------------------------+--------------------------------+--------------------------------+

**When to use:**

- **Particle Filter**: Global localization, kidnapped robot problem, non-Gaussian noise, multi-modal distributions
- **EKF**: Position tracking, Gaussian noise, computational efficiency required


Unscented Kalman Filter (UKF)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Similarities:**

- Both handle non-linear systems without linearization
- Both are suitable for highly non-linear motion/observation models

**Differences:**

+------------------------+--------------------------------+--------------------------------+
| Feature                | Particle Filter                | Unscented Kalman Filter        |
+========================+================================+================================+
| Non-linearity handling | Exact (through sampling)       | Approximation (sigma points)   |
+------------------------+--------------------------------+--------------------------------+
| Computational cost     | High (many particles)          | Moderate (fixed sigma points)  |
+------------------------+--------------------------------+--------------------------------+
| Multi-modal            | Yes                            | No (unimodal Gaussian)         |
+------------------------+--------------------------------+--------------------------------+
| Implementation         | Simple concept                 | More complex mathematics       |
+------------------------+--------------------------------+--------------------------------+

**When to use:**

- **Particle Filter**: Extreme non-linearity, multi-modal distributions, global localization
- **UKF**: Moderate non-linearity, computational efficiency, tracking with good initialization


Histogram Filter
^^^^^^^^^^^^^^^^

**Similarities:**

- Both are non-parametric
- Both can represent multi-modal distributions

**Differences:**

+------------------------+--------------------------------+--------------------------------+
| Feature                | Particle Filter                | Histogram Filter               |
+========================+================================+================================+
| Representation         | Particles (discrete samples)   | Grid cells (discrete bins)     |
+------------------------+--------------------------------+--------------------------------+
| Dimensionality         | Scales better to high          | Suffers from curse of          |
|                        | dimensions                     | dimensionality (grid grows     |
|                        |                                | exponentially)                 |
+------------------------+--------------------------------+--------------------------------+
| Resolution             | Adaptive (particles            | Fixed grid resolution          |
|                        | concentrate in high-           |                                |
|                        | probability regions)           |                                |
+------------------------+--------------------------------+--------------------------------+
| Memory                 | O(N) particles                 | O(cells^dimensions)            |
+------------------------+--------------------------------+--------------------------------+

**When to use:**

- **Particle Filter**: Higher dimensional state spaces (>2D), dynamic resolution needs
- **Histogram Filter**: Low-dimensional problems (1D or 2D), simple implementation


Summary Table
^^^^^^^^^^^^^

+-------------------+---------+-------------+-------------+------------+----------------+
| Method            | Linear  | Multi-modal | Comp. Cost  | Global Loc | Best Use Case  |
+===================+=========+=============+=============+============+================+
| Particle Filter   | No      | Yes         | High        | Yes        | Robust,        |
|                   |         |             |             |            | global         |
+-------------------+---------+-------------+-------------+------------+----------------+
| EKF               | Yes*    | No          | Low         | No         | Tracking,      |
|                   |         |             |             |            | efficient      |
+-------------------+---------+-------------+-------------+------------+----------------+
| UKF               | No      | No          | Medium      | No         | Non-linear     |
|                   |         |             |             |            | tracking       |
+-------------------+---------+-------------+-------------+------------+----------------+
| Histogram         | No      | Yes         | Medium-High | Yes        | Low-dim        |
|                   |         |             |             |            | global loc     |
+-------------------+---------+-------------+-------------+------------+----------------+

\* EKF requires linearization


Reference
~~~~~~~~~~~

- `PROBABILISTIC ROBOTICS <http://www.probabilistic-robotics.org>`_ - Chapters 4 (Bayes Filters) and 8 (MCL)
- `Improving the particle filter in high dimensions using conjugate artificial process noise <https://arxiv.org/pdf/1801.07000>`_
- S. Thrun, W. Burgard, D. Fox, "Probabilistic Robotics", MIT Press, 2005
