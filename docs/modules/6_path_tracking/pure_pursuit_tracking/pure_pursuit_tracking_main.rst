Pure pursuit tracking
---------------------

Path tracking simulation with pure pursuit steering control and PID
speed control.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/pure_pursuit/animation.gif

The red line is a target course, the green cross means the target point
for pure pursuit control, and the blue line is the tracked trajectory.



Algorithm Overview
~~~~~~~~~~~~~~~~~~


Pure Pursuit is a geometric path-tracking algorithm widely used in
autonomous vehicles and mobile robotics. Instead of steering toward the
nearest point on the reference path, the vehicle continuously selects a
target point located ahead on the path (called the *look-ahead point*)
and steers toward it.

At every control step, the algorithm performs three main operations:

1. Select a target point using a look-ahead distance.
2. Compute the angular error between vehicle heading and target point.
3. Compute a steering angle that guides the vehicle toward the target.

The look-ahead distance is dynamically adjusted according to vehicle speed:

.. math::

   L_f = kv + L_{fc}

where :math:`v` is the vehicle speed, :math:`k` is the look-forward gain,
and :math:`L_{fc}` is the minimum look-ahead distance.



Geometric Intuition
~~~~~~~~~~~~~~~~~~~

Pure Pursuit uses the bicycle model, where the rear axle center is treated
as the reference control point. At each control step, a look-ahead point
is selected on the reference path at distance :math:`L_f`.

Let the target point be :math:`(x_t, y_t)` and the rear axle position be
:math:`(x_r, y_r)`. The angle between the vehicle heading and the line
joining the rear axle to the target point is denoted by :math:`\alpha`.

.. math::

\alpha = \tan^{-1}\left(\frac{y_t - y_r}{x_t - x_r}\right) - \theta

where :math:`\theta` is the current vehicle heading angle.

A larger :math:`\alpha` indicates that the target point lies farther away
from the current heading direction, requiring a sharper steering command.



Steering Law Derivation
~~~~~~~~~~~~~~~~~~~~~~~


Under the bicycle model, a vehicle moving with constant steering angle
follows a circular trajectory of radius :math:`R`.

The relation between steering angle :math:`\delta` and turning radius is:

.. math::

   \tan(\delta) = \frac{L}{R}

where :math:`L` is the wheelbase.

Pure Pursuit assumes the vehicle follows a circular arc passing through
the look-ahead point. Using circle geometry, the chord length satisfies:

.. math::

   L_f = 2R \sin(\alpha)

Rearranging gives:

.. math::

   R = \frac{L_f}{2\sin(\alpha)}

Substituting into the bicycle model equation gives the Pure Pursuit
steering law:

.. math::

   \delta = \tan^{-1}\left(\frac{2L\sin(\alpha)}{L_f}\right)

This shows that larger :math:`\alpha` produces sharper turns, while larger
look-ahead distance :math:`L_f` produces smoother steering.



Implementation Details
~~~~~~~~~~~~~~~~~~~~~~

The controller is mainly divided into two stages.

1. Target Point Selection

The function ``search_target_index()`` determines the look-ahead target
point. It first finds the path point nearest to the rear axle position,
then moves forward along the path until the distance exceeds
:math:`L_f`.

2. Steering Computation

The function ``pure_pursuit_steer_control()`` computes the heading error
:math:`\alpha` and uses the steering law above to calculate the steering
command.

The control loop repeatedly performs target selection, steering
computation, and vehicle state updates to smoothly track the reference
path.

Code Link
~~~~~~~~~

.. autofunction:: PathTracking.pure_pursuit.pure_pursuit.pure_pursuit_steer_control

Reference
~~~~~~~~~

- `A Survey of Motion Planning and Control Techniques for Self-driving
  Urban Vehicles <https://arxiv.org/abs/1604.07446>`_


