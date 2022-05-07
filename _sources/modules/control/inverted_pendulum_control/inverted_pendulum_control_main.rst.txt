Inverted Pendulum Control
-----------------------------

An inverted pendulum on a cart consists of a mass :math:`m` at the top of a pole of length :math:`l` pivoted on a
horizontally moving base as shown in the adjacent.

The objective of the control system is to balance the inverted pendulum by applying a force to the cart that the pendulum is attached to.

Modeling
~~~~~~~~~~~~

.. image:: inverted-pendulum.png
    :align: center

- :math:`M`: mass of the cart
- :math:`m`: mass of the load on the top of the rod
- :math:`l`: length of the rod
- :math:`u`: force applied to the cart
- :math:`x`: cart position coordinate
- :math:`\theta`: pendulum angle from vertical

Using Lagrange's equations:

.. math::
    & (M + m)\ddot{x} - ml\ddot{\theta}cos{\theta} + ml\dot{\theta^2}\sin{\theta} = u \\
    & l\ddot{\theta} - g\sin{\theta} = \ddot{x}\cos{\theta}

See this `link <https://en.wikipedia.org/wiki/Inverted_pendulum#From_Lagrange's_equations>`__ for more details.

So

.. math::
    & \ddot{x} =  \frac{m(gcos{\theta} - \dot{\theta}^2l)sin{\theta} + u}{M + m - mcos^2{\theta}} \\
    & \ddot{\theta} = \frac{g(M + m)sin{\theta} - \dot{\theta}^2lmsin{\theta}cos{\theta} + ucos{\theta}}{l(M + m - mcos^2{\theta})}


Linearized model when :math:`\theta` small, :math:`cos{\theta} \approx 1`, :math:`sin{\theta} \approx \theta`, :math:`\dot{\theta}^2 \approx 0`.

.. math::
    & \ddot{x} =  \frac{gm}{M}\theta + \frac{1}{M}u\\
    & \ddot{\theta} = \frac{g(M + m)}{Ml}\theta + \frac{1}{Ml}u

State space:

.. math::
    & \dot{x} = Ax + Bu \\
    & y = Cx + Du

where

.. math::
    & x = [x, \dot{x}, \theta,\dot{\theta}]\\
    & A = \begin{bmatrix}  0 & 1 & 0 & 0\\0 & 0 & \frac{gm}{M} & 0\\0 & 0 & 0 & 1\\0 & 0 & \frac{g(M + m)}{Ml} & 0 \end{bmatrix}\\
    & B = \begin{bmatrix}  0 \\ \frac{1}{M} \\ 0 \\ \frac{1}{Ml} \end{bmatrix}

If control only \theta

.. math::
    & C = \begin{bmatrix} 0 & 0 & 1 & 0 \end{bmatrix}\\
    & D = [0]

If control x and \theta

.. math::
    & C = \begin{bmatrix} 1 & 0 & 0 & 0\\0 & 0 & 1 & 0 \end{bmatrix}\\
    & D = \begin{bmatrix} 0 \\ 0 \end{bmatrix}

LQR control
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The LQR controller minimize this cost function defined as:

.. math::  J = x^T Q x + u^T R u

the feedback control law that minimizes the value of the cost is:

.. math::  u = -K x

where:

.. math::  K = (B^T P B + R)^{-1} B^T P A

and :math:`P` is the unique positive definite solution to the discrete time
`algebraic Riccati equation <https://en.wikipedia.org/wiki/Inverted_pendulum#From_Lagrange's_equations>`__  (DARE):

.. math::  P = A^T P A - A^T P B ( R + B^T P B )^{-1} B^T P A + Q

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Control/InvertedPendulumCart/animation_lqr.gif

MPC control
~~~~~~~~~~~~~~~~~~~~~~~~~~~
The MPC controller minimize this cost function defined as:

.. math:: J = x^T Q x + u^T R u

subject to:

- Linearized Inverted Pendulum model
- Initial state

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Control/InvertedPendulumCart/animation.gif
