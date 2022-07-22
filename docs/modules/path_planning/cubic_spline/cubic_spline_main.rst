Cubic spline planning
---------------------

1D Cubic spline function
~~~~~~~~~~~~~~~~~~~

Cubic spline interpolation is a method of smoothly
interpolating between multiple sample points when given
multiple sample points, as shown in the figure below.

.. image:: spline.png

It separates between each interval between sample points.

The each interval part is approximated by each cubic polynomial.

The cubic spline uses the cubic polynomial equation for interpolation:

:math:`S_j(x)=a_j+b_j(x-x_j)+c_j(x-x_j)^2+d_j(x-x_j)^3`

where :math:`x_j < x < x_{j+1}`, :math:`x_j` is the j-th node of the spline,
:math:`a_j`, :math:`b_j`, :math:`c_j`, :math:`d_j` are the coefficients
of the spline.

As the above equation, there are 4 unknown parameters :math:`(a,b,c,d)` for
one interval, so if the number of sample point is :math:`N`, the
interpolation has :math:`4N` unknown parameters.

The following five conditions are used to determine the 4N unknown parameters:

Constraint 1: Terminal constraints
===================================

:math:`S_j(x_j)=y_j`

This constraint is the terminal constraint of each interval.

The polynomial of each interval will pass through the x,y coordinates of
the sample point.

Constraint 2: Point continuous constraints
============================================

:math:`S_j(x_{j+1})=S_{j+1}(x_{j+1})=y_{j+1}`

This constraint is a continuity condition for the boundary of each interval.
This constraint ensures that the boundary of each interval is continuous.

Constraint 3: Tangent vector continuous constraints
==============================================

:math:`S'_j(x_{j+1})=S'_{j+1}(x_{j+1})`

Constraint 4: Curvature continuous constraints
==============================================

:math:`S''_j(x_{j+1})=S''_{j+1}(x_{j+1})`


Constraint 5: Terminal curvature constraints
========================================================

:math:`S''_0(0)=S''_{n+1}(x_{n})=0`



2D spline path
~~~~~~~~~~~~~~~~~~~

A sample code for cubic path planning.

This code generates a curvature continuous path based on x-y waypoints
with cubic spline.

Heading angle of each point can be also calculated analytically.

.. image:: Figure_1.png
.. image:: Figure_2.png
.. image:: Figure_3.png

