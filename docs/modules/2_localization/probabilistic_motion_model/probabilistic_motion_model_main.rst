Probabilistic motion models
---------------------------

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/probabilistic_motion_model/animation.gif
   :alt: Sampled poses and the closed-form density of the velocity and odometry motion models for straight motion, a left turn and a right turn

A motion model describes where a robot can be after it executes a control.
Because real motion is noisy, the model is a probability distribution
:math:`p(x_t \mid u_t, x_{t-1})` over the new pose
:math:`x_t = (x', y', \theta')`, given the previous pose
:math:`x_{t-1} = (x, y, \theta)` and the control :math:`u_t`.
It is the prediction step of Bayes filters, for example of Monte Carlo
localization (the particle filter).

This example implements the two motion models of chapter 5 of the book
Probabilistic Robotics [1]_. Each model has two forms:

- a **closed form**, which returns how likely a candidate pose :math:`x_t` is,
- a **sampler**, which draws poses from the distribution. A particle filter
  uses the sampler to move its particles.

The animation shows both forms for straight motion, a left turn and a right
turn of 2 seconds. The red dots are poses drawn by the sampler. The gray image
is the closed-form density summed over the heading, and the blue line encloses
90 % of it. The samples form the typical banana shape: an error in the heading
early in the motion moves the robot sideways at the end.

The particle filter example of this repository
moves each particle with one Euler step
and Gaussian noise of fixed covariance on the control. The models of this page
make the noise grow with the speed and the turn rate, and they also provide
the closed form.

Code Link
~~~~~~~~~

.. autofunction:: Localization.probabilistic_motion_model.probabilistic_motion_model.velocity_motion_model

.. autofunction:: Localization.probabilistic_motion_model.probabilistic_motion_model.sample_velocity_motion_model

.. autofunction:: Localization.probabilistic_motion_model.probabilistic_motion_model.odometry_motion_model

.. autofunction:: Localization.probabilistic_motion_model.probabilistic_motion_model.sample_odometry_motion_model

Velocity motion model
~~~~~~~~~~~~~~~~~~~~~

The control is a forward speed :math:`v` and a turn rate :math:`\omega`, held
for a time :math:`\Delta t`. Without noise, the robot moves on a circle of
signed radius :math:`v / \omega` whose center is

.. math::

   x_c = x - \frac{v}{\omega}\sin\theta, \qquad
   y_c = y + \frac{v}{\omega}\cos\theta,

and the new pose is

.. math::

   \begin{pmatrix} x' \\ y' \\ \theta' \end{pmatrix} =
   \begin{pmatrix} x \\ y \\ \theta \end{pmatrix} +
   \begin{pmatrix}
   -\frac{v}{\omega}\sin\theta + \frac{v}{\omega}\sin(\theta + \omega\Delta t) \\
   \frac{v}{\omega}\cos\theta - \frac{v}{\omega}\cos(\theta + \omega\Delta t) \\
   \omega\Delta t
   \end{pmatrix}.

The real speed and turn rate are the commanded ones plus zero-mean normal
noise whose variance grows with the commanded values:

.. math::

   \hat v = v + \varepsilon_{\alpha_1 v^2 + \alpha_2 \omega^2}, \qquad
   \hat \omega = \omega + \varepsilon_{\alpha_3 v^2 + \alpha_4 \omega^2},

where :math:`\varepsilon_{b^2}` is a zero-mean normal variable with variance
:math:`b^2`. Motion on a circle cannot reach every final heading, so the model
adds a final rotation :math:`\hat\gamma = \varepsilon_{\alpha_5 v^2 + \alpha_6 \omega^2}`
and the new heading is :math:`\theta + \hat\omega\Delta t + \hat\gamma\Delta t`.

**Sampler** (Table 5.3 of [1]_): draw :math:`\hat v`, :math:`\hat\omega` and
:math:`\hat\gamma`, then apply the circle motion with :math:`\hat v` and
:math:`\hat\omega`.

**Closed form** (Table 5.1 of [1]_): find the circle that starts at
:math:`(x, y)` with tangent direction :math:`\theta` and passes through
:math:`(x', y')`. Its center is

.. math::

   \mu = \frac{1}{2}\,
   \frac{(x - x')\cos\theta + (y - y')\sin\theta}{(y - y')\cos\theta - (x - x')\sin\theta},
   \qquad
   x^* = \frac{x + x'}{2} + \mu (y - y'), \qquad
   y^* = \frac{y + y'}{2} + \mu (x' - x).

The angle traveled on this circle is
:math:`\Delta\theta = \operatorname{atan2}(y' - y^*, x' - x^*) - \operatorname{atan2}(y - y^*, x - x^*)`,
which gives the speeds that would explain the motion and the final rotation:

.. math::

   \hat\omega = \frac{\Delta\theta}{\Delta t}, \qquad
   \hat v = \hat\omega\,\lambda, \qquad
   \hat\gamma = \frac{\theta' - \theta}{\Delta t} - \hat\omega.

The result is the product of three normal densities:

.. math::

   p(x_t \mid u_t, x_{t-1}) =
   \mathcal{N}(v - \hat v;\, 0, \alpha_1 v^2 + \alpha_2 \omega^2)\,
   \mathcal{N}(\omega - \hat\omega;\, 0, \alpha_3 v^2 + \alpha_4 \omega^2)\,
   \mathcal{N}(\hat\gamma;\, 0, \alpha_5 v^2 + \alpha_6 \omega^2).

All angle differences are wrapped into :math:`[-\pi, \pi)`. When the new
position lies on the line of the start heading, the circle becomes a straight
line, and the code uses :math:`\hat\omega = 0` and :math:`\hat v` equal to the
traveled distance divided by :math:`\Delta t`.

Signed radius
^^^^^^^^^^^^^

In :math:`\hat v = \hat\omega\,\lambda`, :math:`\lambda` is the **signed**
radius of equation 5.17 of [1]_:

.. math::

   \lambda = (y^* - y)\cos\theta - (x^* - x)\sin\theta,

which is positive when the center is on the left of the robot. Line 7 of
Table 5.1 instead multiplies by the distance :math:`r^*` between
:math:`(x, y)` and the center, which is always positive. When :math:`v` and
:math:`\omega` have opposite signs, for example in a forward right turn, the
recovered speed then has the wrong sign. For a noise-free forward right turn
(:math:`v = 1` m/s, :math:`\omega = -0.35` rad/s, :math:`\Delta t = 2` s,
:math:`\alpha_1 = \dots = \alpha_4 = 0.02`, :math:`\alpha_5 = \alpha_6 = 0.01`),
the true end pose gets the density :math:`5.4 \times 10^{-38}` with line 7 as
printed and 26.7 with the signed radius, the same value as for the mirrored
left turn. The test ``test_forward_right_turn`` checks this.

Odometry motion model
~~~~~~~~~~~~~~~~~~~~~

Many robots do not report the executed speeds but the poses that their wheel
odometry integrates. The odometry model uses two consecutive odometry poses
:math:`\bar x_{t-1}` and :math:`\bar x_t` as the control, and splits the
motion between them into a rotation, a translation and a second rotation:

.. math::

   \delta_{rot1} = \operatorname{atan2}(\bar y' - \bar y, \bar x' - \bar x) - \bar\theta, \qquad
   \delta_{trans} = \sqrt{(\bar x' - \bar x)^2 + (\bar y' - \bar y)^2}, \qquad
   \delta_{rot2} = \bar\theta' - \bar\theta - \delta_{rot1}.

Each of the three values gets zero-mean normal noise. The variance of the
rotation noise grows with the rotation (:math:`\alpha_1`) and the translation
(:math:`\alpha_2`). The variance of the translation noise grows with the
translation (:math:`\alpha_3`) and the rotations (:math:`\alpha_4`).

**Sampler** (Table 5.6 of [1]_): subtract noise from the three values of the
odometry and apply them to the previous pose:

.. math::

   x' = x + \hat\delta_{trans}\cos(\theta + \hat\delta_{rot1}), \qquad
   y' = y + \hat\delta_{trans}\sin(\theta + \hat\delta_{rot1}), \qquad
   \theta' = \theta + \hat\delta_{rot1} + \hat\delta_{rot2}.

**Closed form** (Table 5.5 of [1]_): compute the same three values
:math:`\hat\delta_{rot1}, \hat\delta_{trans}, \hat\delta_{rot2}` for the
motion from :math:`x_{t-1}` to the candidate pose :math:`x_t`, and multiply
the three normal densities of the differences:

.. math::

   p(x_t \mid u_t, x_{t-1}) =
   \mathcal{N}(\delta_{rot1} - \hat\delta_{rot1};\, 0, \alpha_1 \hat\delta_{rot1}^2 + \alpha_2 \hat\delta_{trans}^2)\,
   \mathcal{N}(\delta_{trans} - \hat\delta_{trans};\, 0, \alpha_3 \hat\delta_{trans}^2 + \alpha_4 (\hat\delta_{rot1}^2 + \hat\delta_{rot2}^2))\,
   \mathcal{N}(\delta_{rot2} - \hat\delta_{rot2};\, 0, \alpha_1 \hat\delta_{rot2}^2 + \alpha_2 \hat\delta_{trans}^2).

For a translation shorter than 1 cm, the direction of travel is undefined, so
the code sets :math:`\delta_{rot1} = 0` and the whole turn goes into
:math:`\delta_{rot2}`.

A density over the noise, not over the pose
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Both closed forms multiply densities of the noise variables:
:math:`(v, \omega, \gamma)` for the velocity model and
:math:`(\delta_{rot1}, \delta_{trans}, \delta_{rot2})` for the odometry model.
They do not include the Jacobian of the change of variables to the pose
:math:`(x', y', \theta')`, so they are not a density over the pose and do not
integrate to one over :math:`(x', y', \theta')`. For example, take a motion of
2 m with a left turn from the origin (velocity model: :math:`v = 1` m/s,
:math:`\omega = 0.35` rad/s, :math:`\Delta t = 2` s; odometry model:
:math:`\delta_{rot1} = 0.35` rad, :math:`\delta_{trans} = 2` m,
:math:`\delta_{rot2} = 0.35` rad), :math:`\alpha_1 = \dots = \alpha_4 = 0.02`
and :math:`\alpha_5 = \alpha_6 = 0.01`. Summed on a grid of
:math:`141 \times 141 \times 241` cells over
:math:`[-0.5, 3] \times [-1, 2.5]` m and all headings, the closed forms give
about 7.6 for the velocity model and 2.2 for the odometry model.

For the odometry model, the Jacobian determinant of the map from
:math:`(\delta_{rot1}, \delta_{trans}, \delta_{rot2})` to
:math:`(x', y', \theta')` is :math:`\hat\delta_{trans}`. Dividing by it gives
a density over the pose, which sums to about 1.05 on the same grid, close to
one. The test ``test_odometry_density_over_pose_integrates_to_one`` checks
this.

In a particle filter, this does not change the result, because the weights
are normalized. It matters when the output is used as a probability density
over the pose, for example to compare it with a histogram of samples.

The sampler and the closed form of the odometry model also differ slightly:
Table 5.6 computes the noise variance from the measured odometry
:math:`\delta`, but Table 5.5 computes it from the candidate motion
:math:`\hat\delta`. So the two forms describe close, but not identical,
distributions. With the parameters above, about 93 % of the odometry samples
fall inside the 90 % region of the closed form (after the Jacobian). If the
closed form takes the variance from the measured odometry instead, the share
is 90 %. The velocity model, whose variance depends only on the commanded
control in both tables, also gives 90 %.

References
~~~~~~~~~~

.. [1] S. Thrun, W. Burgard and D. Fox, *Probabilistic Robotics*, MIT Press,
   2005, chapter 5 (Robot Motion).
