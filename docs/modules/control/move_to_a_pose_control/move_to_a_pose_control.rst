Move to a pose control
----------------------

This is a simulation of moving to a pose control.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/move_to_pose/animation.gif


PathFinderController class
~~~~~~~~~~~~~~~~~~~~~~~~~~

Constructor
~~~~~~~~~~~

.. code-block:: ipython3

   PathFinderController(Kp_rho, Kp_alpha, Kp_beta)

Constructs an instantiate of the PathFinderController for navigating a 3-DOF wheeled robot on a 2D plane.

Parameters:

- | :math:`Kp_{rho}` : The linear velocity gain to translate the robot along a line towards the goal
- | :math:`Kp_alpha` : The angular velocity gain to rotate the robot towards the goal
- | :math:`Kp_beta` : The offset angular velocity gain accounting for smooth merging to the goal angle (i.e., it helps the robot heading to be parallel to the target angle.)


Member function(s)
~~~~~~~~~~~~~~~~~~

.. code-block:: ipython3

   calc_control_command(x_diff, y_diff, theta, theta_goal)

Returns the control command for the linear and angular velocities as well as the distance to goal

Parameters:

- | :math:`x_diff` : The position of target with respect to current robot position in x direction
- | :math:`y_diff` : The position of target with respect to current robot position in y direction
- | :math:`theta` : The current heading angle of robot with respect to x axis
- | :math:`theta_goal` : The target angle of robot with respect to x axis

Returns:

- | :math:`rho` : The distance between the robot and the goal position
- | :math:`v` : Command linear velocity
- | :math:`w` : Command angular velocity

Move to a Pose Robot (Class)
----------------------------
This program (move_to_pose_robot.py) provides a Robot class to define different robots with different specifications. 
Using this class, you can simulate different robots simultaneously and compare the effect of your parameter settings.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/move_to_pos_robot_class/animation.gif

Note: The robot class is based on PathFinderController class in 'the move_to_pose.py'.

Robot Class
~~~~~~~~~~~

Constructor
~~~~~~~~~~~

.. code-block:: ipython3

    Robot(name, color, max_linear_speed, max_angular_speed, path_finder_controller)

Constructs an instantiate of the 3-DOF wheeled Robot navigating on a 2D plane

Parameters:

- | :math:`name` : (string) The name of the robot
- | :math:`color` : (string) The color of the robot
- | :math:`max_linear_speed` : (float) The maximum linear speed that the robot can go
- | :math:`max_angular_speed` : (float) The maximum angular speed that the robot can rotate about its vertical axis
- | :math:`path_finder_controller` : (PathFinderController) A configurable controller to finds the path and calculates command linear and angular velocities.

Member function(s)
~~~~~~~~~~~~~~~~~~

.. code-block:: ipython3

    set_start_target_poses(pose_start, pose_target)

Sets the start and target positions of the robot.

Parameters:

- | :math:`pose_start` : (Pose) Start postion of the robot (see the Pose class)
- | :math:`pose_target` : (Pose) Target postion of the robot (see the Pose class)

.. code-block:: ipython3

    move(dt)

Move the robot for one time step increment

Parameters:

- | :math:`dt` : <float> time increment

See Also 
--------
- PathFinderController class


Ref:
----
-  `P. I. Corke, "Robotics, Vision and Control" \| SpringerLink
   p102 <https://link.springer.com/book/10.1007/978-3-642-20144-8>`__
