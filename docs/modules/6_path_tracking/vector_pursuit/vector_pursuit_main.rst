Vector Pursuit
==============

Vector Pursuit uses both the position and tangent heading of a look-ahead
point. It combines two instantaneous screw motions: travel along a circle
to the point, and correction of the remaining heading difference.

.. image:: https://raw.githubusercontent.com/AtsushiSakai/PythonRoboticsGifs/f379e41f9ba6aad4ada89139392e48325bfdc512/PathTracking/vector_pursuit/animation.gif
   :alt: Bicycle model following an S-shaped path using Vector Pursuit

Curvature from a target pose
----------------------------

Express the target in the rear-axle frame as :math:`(x_t,y_t,\theta)`,
where :math:`\theta` is the wrapped target-minus-vehicle heading.
This example uses positive counterclockwise angles and targets with
:math:`x_t>0`.

The chord length, signed circular-arc angle, and arc length are

.. math::

   d=\sqrt{x_t^2+y_t^2},\qquad
   \beta=2\operatorname{atan2}(y_t,x_t),\qquad
   \ell=\frac{d}{\operatorname{sinc}(\beta/2)},

where :math:`\operatorname{sinc}(z)=\sin(z)/z` and
:math:`\operatorname{sinc}(0)=1`.
For forward speed :math:`v`, translation takes :math:`t_t=\ell/v`.
Setting rotation-correction time :math:`t_r=kt_t`, with :math:`k>0`, gives

.. math::

   \omega_t=\frac{v\beta}{\ell},\qquad
   \omega_r=\frac{v(\theta-\beta)}{k\ell},\qquad
   \kappa=\frac{\omega_t+\omega_r}{v}
          =\frac{(k-1)\beta+\theta}{k\ell}.

This is the curvature form of the paper's combined-screw construction.
It also handles :math:`y_t=0`, yielding :math:`\kappa=\theta/(kd)`.
A zero numerator means straight motion. Increasing :math:`k` reduces the
heading-correction contribution; the limit is pure-pursuit curvature
:math:`2y_t/d^2`.

The bicycle steering command is

.. math::

   \delta=\operatorname{clip}\bigl(\arctan(L\kappa),
                  -\delta_{\max},\delta_{\max}\bigr).

Here :math:`L` is the wheelbase. The kinematic model obeys
:math:`\dot x=v\cos\psi`, :math:`\dot y=v\sin\psi`, and
:math:`\dot\psi=v\tan\delta/L`.

Example
-------

The sample uses a cubic-spline path, a 4 m look-ahead along its sampled
arc length, :math:`k=2`, and proportional speed control. The blue line is
the rear-axle trajectory; blue and magenta segments indicate vehicle and
target headings. The lower panel shows steering and its limits.

This forward-only example requires the target to stay ahead. It does not
support reverse driving, self-intersecting routes, obstacle avoidance, or
final-pose stabilization. Selecting the final target does not finish the
simulation: the rear axle must reach the goal tolerance.

Code
----

.. autofunction:: PathTracking.vector_pursuit.vector_pursuit.main

.. autofunction:: PathTracking.vector_pursuit.vector_pursuit.vector_pursuit_curvature

.. autofunction:: PathTracking.vector_pursuit.vector_pursuit.simulate

References
----------

- J. S. Wit, `Vector Pursuit Path Tracking for Autonomous Ground Vehicles
  <https://apps.dtic.mil/sti/tr/pdf/ADA468928.pdf>`__, PhD dissertation,
  University of Florida, 2000.
- J. Wit, C. D. Crane III, D. Armstrong, `Autonomous Ground Vehicle Path Tracking
  <https://doi.org/10.1002/rob.20031>`__, Journal of Robotic Systems 21(8),
  439–449, 2004, equations 20–30.
