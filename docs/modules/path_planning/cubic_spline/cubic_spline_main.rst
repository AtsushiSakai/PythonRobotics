Cubic spline planning
---------------------

1D Cubic spline function
~~~~~~~~~~~~~~~~~~~

Cubic spline interpolation is a method of smoothly
interpolating between multiple data points when given
multiple data points, as shown in the figure below.

.. image:: spline.png

It separates between each interval between data points.

The each interval part is approximated by each cubic polynomial.

The cubic spline uses the cubic polynomial equation for interpolation:

:math:`S_j(x)=a_j+b_j(x-x_j)+c_j(x-x_j)^2+d_j(x-x_j)^3`

where :math:`x_j < x < x_{j+1}`, :math:`x_j` is the j-th node of the spline,
:math:`a_j`, :math:`b_j`, :math:`c_j`, :math:`d_j` are the coefficients
of the spline.

As the above equation, there are 4 unknown parameters :math:`(a,b,c,d)` for
one interval, so if the number of data points is :math:`N`, the
interpolation has :math:`4N` unknown parameters.

The following five conditions are used to determine the :math:`4N`
unknown parameters:

Constraint 1: Terminal constraints
===================================

:math:`S_j(x_j)=y_j`

This constraint is the terminal constraint of each interval.

The polynomial of each interval will pass through the x,y coordinates of
the data points.

Constraint 2: Point continuous constraints
============================================

:math:`S_j(x_{j+1})=S_{j+1}(x_{j+1})=y_{j+1}`

This constraint is a continuity condition for the boundary of each interval.

This constraint ensures that the boundary of each interval is continuous.

Constraint 3: Tangent vector continuous constraints
==============================================

:math:`S'_j(x_{j+1})=S'_{j+1}(x_{j+1})`

This constraint is a continuity condition for the first derivative of
the boundary of each interval.

This constraint makes the vectors of the boundaries of each
interval continuous.


Constraint 4: Curvature continuous constraints
==============================================

:math:`S''_j(x_{j+1})=S''_{j+1}(x_{j+1})`

This constraint is the continuity condition for the second derivative of
the boundary of each interval.

This constraint makes the curvature of the boundaries of each
interval continuous.


Constraint 5: Terminal curvature constraints
========================================================

:math:`S''_0(0)=S''_{n+1}(x_{n})=0`.

The constraint is a boundary condition for the second derivative of the starting and ending points.

Our sample code assumes these terminal curvatures are 0, which is well known as Natural Cubic Spline.

How to calculate the unknown parameters :math:`a_j, b_j, c_j, d_j`
===================================================================

Step1: calculate :math:`a_j`
+++++++++++++++++++++++++++++

Spline coefficients :math:`a_j` can be calculated by y positions of the data points:

:math:`a_j = y_i`.

Step2: calculate :math:`c_j`
+++++++++++++++++++++++++++++

Spline coefficients :math:`c_j` can be calculated by solving the linear equation:

:math:`Ac_j = B`.

The matrix :math:`A` and :math:`B` are defined as follows:

.. math::

	A=\left[\begin{array}{cccccc}
	1 & 0 & 0 & 0 & \cdots & 0 \\
	h_{0} & 2\left(h_{0}+h_{1}\right) & h_{1} & 0 & \cdots & 0 \\
	0 & h_{1} & 2\left(h_{1}+h_{2}\right) & h_{2} & \cdots & 0 \\
	0 & 0 & h_{2} & 2\left(h_{2}+h_{3}\right) & \cdots & 0 \\
	0 & 0 & 0 & h_{3} & \ddots & \\
	\vdots & \vdots & & & & \\
	0 & 0 & 0 & \cdots & 0 & 1
	\end{array}\right]

.. math::
	B=\left[\begin{array}{c}
	0 \\
	\frac{3}{h_{1}}\left(a_{2}-a_{1}\right)-\frac{3}{h_{0}}\left(a_{1}-a_{0}\right) \\
	\vdots \\
	\frac{3}{h_{n-1}}\left(a_{n}-a_{n-1}\right)-\frac{3}{h_{n-2}}\left(a_{n-1}-a_{n-2}\right) \\
	0
	\end{array}\right]

where :math:`h_{i}` is the x position distance between the i-th and (i+1)-th data points.

Step3: calculate :math:`d_j`
+++++++++++++++++++++++++++++

Spline coefficients :math:`d_j` can be calculated by the following equation:

:math:`d_{j}=\frac{c_{j+1}-c_{j}}{3 h_{j}}`

Step4: calculate :math:`b_j`
+++++++++++++++++++++++++++++

Spline coefficients :math:`b_j` can be calculated by the following equation:

:math:`b_{i}=\frac{1}{h_i}(a_{i+1}-a_{i})-\frac{h_i}{3}(2c_{i}+c_{i+1})`

API
===

These are 1D cubic spline interpolation APIs:

.. autoclass:: PathPlanning.CubicSpline.cubic_spline_planner.CubicSpline1D
	:members:

.. autofunction:: PathPlanning.CubicSpline.cubic_spline_planner.CubicSpline1D
.. autofunction:: PathPlanning.CubicSpline.cubic_spline_planner.CubicSpline1D.calc_position
.. autofunction:: PathPlanning.CubicSpline.cubic_spline_planner.CubicSpline1D.calc_first_derivative
.. autofunction:: PathPlanning.CubicSpline.cubic_spline_planner.CubicSpline1D.calc_second_derivative

2D spline path
~~~~~~~~~~~~~~~~~~~

A sample code for cubic path planning.

This code generates a curvature continuous path based on x-y waypoints
with cubic spline.

Heading angle of each point can be also calculated analytically.

.. image:: Figure_1.png
.. image:: Figure_2.png
.. image:: Figure_3.png

API
===

.. autofunction:: PathPlanning.CubicSpline.cubic_spline_planner.CubicSpline2D

References
~~~~~~~~~~
-  `Cubic Splines James Keesling <https://people.clas.ufl.edu/kees/files/CubicSplines.pdf>`__

