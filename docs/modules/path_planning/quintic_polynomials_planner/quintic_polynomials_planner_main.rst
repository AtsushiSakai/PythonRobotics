
Quintic polynomials planning
----------------------------

Motion planning with quintic polynomials.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/QuinticPolynomialsPlanner/animation.gif

It can calculate 2D path, velocity, and acceleration profile based on
quintic polynomials.



Quintic polynomials for one dimensional robot motion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We assume a one-dimensional robot motion :math:`x(t)` at time :math:`t` is
formulated as a quintic polynomials based on time as follows:

.. math:: x(t) = a_0+a_1t+a_2t^2+a_3t^3+a_4t^4+a_5t^5
    :label: quintic_eq1

:math:`a_0, a_1. a_2, a_3, a_4, a_5` are parameters of the quintic polynomial.

It is assumed that terminal states (start and end) are known as boundary conditions.

Start position, velocity, and acceleration are :math:`x_s, v_s, a_s` respectively.

End position, velocity, and acceleration are :math:`x_e, v_e, a_e` respectively.

So, when time is 0.

.. math:: x(0) = a_0 = x_s
    :label: quintic_eq2

Then, differentiating the equation :eq:`quintic_eq1` with t,

.. math:: x'(t) = a_1+2a_2t+3a_3t^2+4a_4t^3+5a_5t^4
    :label: quintic_eq3

So, when time is 0,

.. math:: x'(0) = a_1 = v_s
    :label: quintic_eq4

Then, differentiating the equation :eq:`quintic_eq3` with t again,

.. math:: x''(t) = 2a_2+6a_3t+12a_4t^2
    :label: quintic_eq5

So, when time is 0,

.. math:: x''(0) = 2a_2 = a_s
    :label: quintic_eq6

so, we can calculate :math:`a_0, a_1, a_2` with eq. :eq:`quintic_eq2`, :eq:`quintic_eq4`, :eq:`quintic_eq6` and boundary conditions.

:math:`a_3, a_4, a_5` are still unknown in eq :eq:`quintic_eq1`.

We assume that the end time for a maneuver is :math:`T`, we can get these equations from eq :eq:`quintic_eq1`, :eq:`quintic_eq3`, :eq:`quintic_eq5`:

.. math:: x(T)=a_0+a_1T+a_2T^2+a_3T^3+a_4T^4+a_5T^5=x_e
    :label: quintic_eq7

.. math:: x'(T)=a_1+2a_2T+3a_3T^2+4a_4T^3+5a_5T^4=v_e
    :label: quintic_eq8

.. math:: x''(T)=2a_2+6a_3T+12a_4T^2+20a_5T^3=a_e
    :label: quintic_eq9

From eq :eq:`quintic_eq7`, :eq:`quintic_eq8`, :eq:`quintic_eq9`, we can calculate :math:`a_3, a_4, a_5` to solve the linear equations: :math:`Ax=b`

.. math:: \begin{bmatrix} T^3 & T^4 & T^5 \\ 3T^2 & 4T^3 & 5T^4 \\ 6T & 12T^2 & 20T^3 \end{bmatrix}\begin{bmatrix} a_3\\ a_4\\ a_5\end{bmatrix}=\begin{bmatrix} x_e-x_s-v_sT-0.5a_sT^2\\ v_e-v_s-a_sT\\ a_e-a_s\end{bmatrix}

We can get all unknown parameters now.

Quintic polynomials for two dimensional robot motion (x-y)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you use two quintic polynomials along x axis and y axis, you can plan for two dimensional robot motion in x-y plane.

.. math:: x(t) = a_0+a_1t+a_2t^2+a_3t^3+a_4t^4+a_5t^5
    :label: quintic_eq10

.. math:: y(t) = b_0+b_1t+b_2t^2+b_3t^3+b_4t^4+b_5t^5
    :label: quintic_eq11

It is assumed that terminal states (start and end) are known as boundary conditions.

Start position, orientation, velocity, and acceleration are :math:`x_s, y_s, \theta_s, v_s, a_s` respectively.

End position, orientation, velocity, and acceleration are :math:`x_e, y_e. \theta_e, v_e, a_e` respectively.

Each velocity and acceleration boundary condition can be calculated with each orientation.

:math:`v_{xs}=v_scos(\theta_s), v_{ys}=v_ssin(\theta_s)`

:math:`v_{xe}=v_ecos(\theta_e), v_{ye}=v_esin(\theta_e)`

References:
~~~~~~~~~~~

-  `Local Path Planning And Motion Control For Agv In
   Positioning <http://ieeexplore.ieee.org/document/637936/>`__


