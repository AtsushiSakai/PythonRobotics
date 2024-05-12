.. _linearquadratic-regulator-(lqr)-steering-control:

Linear–quadratic regulator (LQR) steering control
-------------------------------------------------

Path tracking simulation with LQR steering control and PID speed
control.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/lqr_steer_control/animation.gif

Overview
~~~~~~~~

The LQR (Linear Quadratic Regulator) steering control model implemented in lqr_steer_control.py provides a method for autonomous vehicles to track a desired trajectory by adjusting steering angle based on feedback from the current state and the desired trajectory. This model utilizes a combination of PID speed control and LQR steering control to achieve smooth and accurate trajectory tracking.


State Representation
~~~~~~~~~~~~~~~~~~~~

The state of the vehicle is represented by its position (x, y), orientation yaw, and velocity v.

Control Inputs
~~~~~~~~~~~~~~

The control inputs to the system are the acceleration a and steering angle delta.

Motion Model
~~~~~~~~~~~~



Model Parameters:

Various parameters are used in the model, including the wheelbase L of the vehicle, the maximum steering angle max_steer, time step dt, and speed proportional gain Kp.
LQR Parameters:

The LQR controller is parameterized by the matrices Q and R, which define the cost function for the state and control inputs, respectively.
Implementation Details:
The model calculates the optimal steering angle delta using LQR control based on the current state and the desired trajectory.
It linearizes the dynamics of the system around the current state to obtain the state-space representation.
The discrete-time Algebraic Riccati equation (DARE) is solved to obtain the optimal control gain.
The control input delta is determined by a combination of feedforward and feedback components.
Additionally, a PID controller is used to adjust the vehicle's speed to match the desired speed profile along the trajectory.
Usage:
The model can be integrated into autonomous vehicle control systems for path tracking applications.
Input parameters include the desired trajectory (cx, cy, cyaw, ck), speed profile, and simulation parameters.
The main() function demonstrates the usage of the model by simulating the vehicle's trajectory tracking behavior.
Derivation Steps for the Linearized A, B Matrix
The linearized A, B matrix is derived to represent the state-space model of the vehicle dynamics. Here are the steps involved:

State Space Representation:

Define the state vector x comprising position, orientation, and velocity.
Define the control input vector u comprising acceleration and steering angle.
State Transition Model:

Formulate the state transition equation x_next = A * x + B * u to represent the dynamics of the system.
Linearization:

Linearize the state transition equation around the current state using Taylor series expansion.
Obtain the Jacobian matrices A and B, representing the partial derivatives of the state transition equation with respect to the state and control inputs, respectively.
Discretization:

Convert the continuous-time state-space model to discrete-time by discretizing the dynamics using appropriate integration methods.
LQR Control Design:

Apply the LQR control design methodology to obtain the optimal control gain matrix K by solving the discrete-time Algebraic Riccati equation (DARE).
Implementation:

Utilize the obtained matrices A, B, and K in the control algorithm to compute the optimal control input for the given state and desired trajectory.

By following these steps, the linearized A, B matrix can be derived and integrated into the control algorithm to achieve effective trajectory tracking performance.


References:
~~~~~~~~~~~
-  `ApolloAuto/apollo: An open autonomous driving platform <https://github.com/ApolloAuto/apollo>`_

