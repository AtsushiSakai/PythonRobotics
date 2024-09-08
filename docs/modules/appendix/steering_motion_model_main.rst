
Steering Motion Model
-----------------------

Turning radius calculation by steering motion model
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The turning Radius represents the radius of the circle when the robot turns, as shown in the diagram below.

.. image:: steering_motion_model/steering_model.png

When the steering angle is tilted by :math:`θ`,
the turning radius :math:`R` can be calculated using the following equation,
based on the geometric relationship between the wheelbase (WB),
which is the distance between the rear wheel center and the front wheel center,
and the assumption that the turning radius circle passes through the center of
the rear wheels in the diagram above.

:math:`R = WB / tan(θ)`

Turning radius calculation by 2 consecutive positions of the robot trajectory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In this section, we will derive the formula for the turning radius from 2 consecutive positions of the robot trajectory.

.. image:: steering_motion_model/turning_radius_calc1.png

As shown in the upper diagram above, the robot moves from a point at time :math:`t` to a point at time :math:`t+1`.
Each point is represented by a 2D position :math:`(x_t, y_t)` and an orientation :math:`\theta_t`.

The distance between the two points is :math:`d = \sqrt{(x_{t+1} - x_t)^2 + (y_{t+1} - y_t)^2}`.

The angle between the two vectors from the turning center to the two points is :math:`\theta = \theta_{t+1} - \theta_t`.
Here, by drawing a perpendicular line from the center of the turning radius
to a straight line of length :math:`d` connecting two points,
the following equation can be derived from the resulting right triangle.

:math:`sin\frac{\theta}{2} = \frac{d}{2R}`

So, the turning radius :math:`R` can be calculated as follows.

:math:`R = \frac{d}{2sin\frac{\theta}{2}}`

The curvature :math:`\kappa` is the reciprocal of the turning radius.
So, the curvature can be calculated as follows.

:math:`\kappa = \frac{2sin\frac{\theta}{2}}{d}`

References:
~~~~~~~~~~~

- `Vehicle Dynamics and Control <https://link.springer.com/book/10.1007/978-1-4614-1433-9>`_
