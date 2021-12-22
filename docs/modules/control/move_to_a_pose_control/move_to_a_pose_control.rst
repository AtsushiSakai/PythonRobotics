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

Constructs an instantiate of the controller.

Parameters:
- Kp_rho: The linear velocity gain to translate the robot along a line towards the goal
- Kp_alpha: The angular velocity gain to rotate the robot towards the goal
- Kp_beta: The offset angular velocity gain accounting for smooth merging to the goal angle


Member function(s)
~~~~~~~~~~~~~~~~~~

.. code-block:: ipython3
   calc_control_command(x_diff, y_diff, theta, theta_goal)

Returns the control command for the linear and angular velocities as well as the distance to goal (rho)

Parameters:
- x_diff: The position of target with respect to current robot position in x direction
- y_diff: The position of target with respect to current robot position in y direction
- theta: The current heading angle of robot with respect to x axis
- theta_goal: The target angle of robot with respect to x axis

Returns:
- The distance between the robot and the goal position
- Command linear velocity
- Command angular velocity

Move to a Pose Control (Advanced)
---------------------------------
A more advanced version of the move_to_pose controller is provided in move_to_pose_robot_class.py.

.. image:: https://user-images.githubusercontent.com/93126501/145834505-a8df8311-5445-413f-a96f-00460d47991c.png

This program implemented a Robot class that helps defining different robots with different specifications and simulating them simultaneously.

Robot Class
~~~~~~~~~~~

Constructor
~~~~~~~~~~~

.. code-block:: ipython3
    Robot(name, color, max_linear_speed, max_angular_speed, path_finder_controller)

Member function(s)
~~~~~~~~~~~~~~~~~~

.. code-block:: ipython3
    set_start_target_poses(pose_start, pose_target)

Sets the start and target positions of the robot.

Parameters:
- pose_start: <Pose> Start postion of the robot (see the Pose class)
- pose_target: <Pose> Target postion of the robot (see the Pose class)

.. code-block:: ipython3
    move(dt)

Move the robot for a time increment

Parameters:
- dt: <float> time increment

Ref:
----
-  `P. I. Corke, "Robotics, Vision and Control" \| SpringerLink
   p102 <https://link.springer.com/book/10.1007/978-3-642-20144-8>`__
