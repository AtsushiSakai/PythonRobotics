Move to a Pose Control
----------------------

In this section, we present the logic of PathFinderController that drives a car from a start pose (x, y, theta) to a goal pose. A simulation of moving to a pose control is presented below.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/move_to_pose/animation.gif

Position Control of non-Holonomic Systems
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This section explains the logic of a position controller for systems with constraint (non-Holonomic system).

The position control of a 1-DOF (Degree of Freedom) system is quite straightforward. We only need to compute a position error and multiply it with a proportional gain to create a command. The actuator of the system takes this command and drive the system to the target position. This controller can be easily extended to higher dimensions (e.g., using Kp_x and Kp_y gains for a 2D position control). In these systems, the number of control commands is equal to the number of degrees of freedom (Holonomic system). 

To describe the configuration of a car on a 2D plane, we need three DOFs (i.e., x, y, and theta). But to drive a car we only need two control commands (theta_engine and theta_steering_wheel). This difference is because of a constraint between the x and y DOFs. The relationship between the delta_x and delta_y is governed by the theta_steering_wheel.

Note that a car is normally a non-Holonomic system but if the road is slippery, the car turns into a Holonomic system and thus it needs three independent commands to be controlled.

PathFinderController class
~~~~~~~~~~~~~~~~~~~~~~~~~~

Constructor
^^^^^^^^^^^

.. code-block:: ipython3

   PathFinderController(Kp_rho, Kp_alpha, Kp_beta)

Constructs an instantiate of the PathFinderController for navigating a 3-DOF wheeled robot on a 2D plane.

Parameters:

- | **Kp_rho** : The linear velocity gain to translate the robot along a line towards the goal
- | **Kp_alpha** : The angular velocity gain to rotate the robot towards the goal
- | **Kp_beta** : The offset angular velocity gain accounting for smooth merging to the goal angle (i.e., it helps the robot heading to be parallel to the target angle.)


Member function(s)
^^^^^^^^^^^^^^^^^^

.. code-block:: ipython3

   calc_control_command(x_diff, y_diff, theta, theta_goal)

Returns the control command for the linear and angular velocities as well as the distance to goal

Parameters:

- | **x_diff** : The position of target with respect to current robot position in x direction
- | **y_diff** : The position of target with respect to current robot position in y direction
- | **theta** : The current heading angle of robot with respect to x axis
- | **theta_goal** : The target angle of robot with respect to x axis

Returns:

- | **rho** : The distance between the robot and the goal position
- | **v** : Command linear velocity
- | **w** : Command angular velocity

How does the Algorithm Work
~~~~~~~~~~~~~~~~~~~~~~~~~~~
The distance between the robot and the goal position, :math:`\rho`, is computed as

.. math::
 \rho = \sqrt{(x_{robot} - x_{target})^2 + (y_{robot} - y_{target})^2}.

The distance :math:`\rho` is used to determine the robot speed. The idea is to slow down the robot as it gets closer to the target.

.. math::
 v = K_P{_\rho} \times \rho\qquad
 :label: move_to_a_pose_eq1

Note that for your applications, you need to tune the speed gain, :math:`K_P{_\rho}` to a proper value.

To turn the robot and align its heading, :math:`\theta`, toward the target position (not orientation),  :math:`\rho \vec{u}`, we need to compute the angle difference :math:`\alpha`. 

.. math::
 \alpha = (\arctan2(y_{diff}, x_{diff}) - \theta + \pi) mod (2\pi) - \pi

The term :math:`mod(2\pi)` is used to map the angle to :math:`[-\pi, \pi)` range.

Lastly to correct the orientation of the robot, we need to compute the orientation error, :math:`\beta`, of the robot.

.. math::
 \beta = (\theta_{goal} - \theta - \alpha + \pi) mod (2\pi) - \pi

Note that to cancel out the effect of :math:`\alpha` when the robot is at the vicinity of the target, the term 

:math:`-\alpha` is included.

The final angular speed command is given by

.. math::
 \omega = K_P{_\alpha} \alpha - K_P{_\beta} \beta\qquad
 :label: move_to_a_pose_eq2
 
The linear and angular speeds (Equations :eq:`move_to_a_pose_eq1` and :eq:`move_to_a_pose_eq2`) are the output of the algorithm.

Move to a Pose Robot (Class)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This program (move_to_pose_robot.py) provides a Robot class to define different robots with different specifications. 
Using this class, you can simulate different robots simultaneously and compare the effect of your parameter settings.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Control/move_to_pos_robot_class/animation.gif

Note: The robot class is based on PathFinderController class in 'the move_to_pose.py'.

Robot Class
^^^^^^^^^^^^

Constructor
^^^^^^^^^^^^

.. code-block:: ipython3

    Robot(name, color, max_linear_speed, max_angular_speed, path_finder_controller)

Constructs an instantiate of the 3-DOF wheeled Robot navigating on a 2D plane

Parameters:

- | **name** : (string) The name of the robot
- | **color** : (string) The color of the robot
- | **max_linear_speed** : (float) The maximum linear speed that the robot can go
- | **max_angular_speed** : (float) The maximum angular speed that the robot can rotate about its vertical axis
- | **path_finder_controller** : (PathFinderController) A configurable controller to finds the path and calculates command linear and angular velocities.

Member function(s)
^^^^^^^^^^^^^^^^^^^

.. code-block:: ipython3

    set_start_target_poses(pose_start, pose_target)

Sets the start and target positions of the robot.

Parameters:

- | **pose_start** : (Pose) Start postion of the robot (see the Pose class)
- | **pose_target** : (Pose) Target postion of the robot (see the Pose class)

.. code-block:: ipython3

    move(dt)

Move the robot for one time step increment

Parameters:

- | **dt** : <float> time increment



References
~~~~~~~~~~~~
- PathFinderController class
-  `P. I. Corke, "Robotics, Vision and Control" \| SpringerLink
   p102 <https://link.springer.com/book/10.1007/978-3-642-20144-8>`__
