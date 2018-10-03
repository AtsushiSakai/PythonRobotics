.. _path_tracking:

Path tracking
=============

move to a pose control
----------------------

This is a simulation of moving to a pose control

|2|

Ref:

-  `P. I. Corke, "Robotics, Vision and Control" \| SpringerLink
   p102 <https://link.springer.com/book/10.1007/978-3-642-20144-8>`__

Pure pursuit tracking
---------------------

Path tracking simulation with pure pursuit steering control and PID
speed control.

|3|

The red line is a target course, the green cross means the target point
for pure pursuit control, the blue line is the tracking.

Ref:

-  `A Survey of Motion Planning and Control Techniques for Self-driving
   Urban Vehicles <https://arxiv.org/abs/1604.07446>`__

Stanley control
---------------

Path tracking simulation with Stanley steering control and PID speed
control.

|4|

Ref:

-  `Stanley: The robot that won the DARPA grand
   challenge <http://robots.stanford.edu/papers/thrun.stanley05.pdf>`__

-  `Automatic Steering Methods for Autonomous Automobile Path
   Tracking <https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf>`__

Rear wheel feedback control
---------------------------

Path tracking simulation with rear wheel feedback steering control and
PID speed control.

|5|

Ref:

-  `A Survey of Motion Planning and Control Techniques for Self-driving
   Urban Vehicles <https://arxiv.org/abs/1604.07446>`__

.. _linearquadratic-regulator-(lqr)-steering-control:

Linear–quadratic regulator (LQR) steering control
-------------------------------------------------

Path tracking simulation with LQR steering control and PID speed
control.

|6|

Ref:

-  `ApolloAuto/apollo: An open autonomous driving
   platform <https://github.com/ApolloAuto/apollo>`__

.. _linearquadratic-regulator-(lqr)-speed-and-steering-control:

Linear–quadratic regulator (LQR) speed and steering control
-----------------------------------------------------------

Path tracking simulation with LQR speed and steering control.

|7|

Ref:

-  `Towards fully autonomous driving: Systems and algorithms - IEEE
   Conference
   Publication <http://ieeexplore.ieee.org/document/5940562/>`__

Model predictive speed and steering control
-------------------------------------------

Path tracking simulation with iterative linear model predictive speed
and steering control.

.. raw:: html

   <img src="https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathTracking/model_predictive_speed_and_steer_control/animation.gif" width="640">

Ref:

-  `notebook <https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/model_predictive_speed_and_steer_control/notebook.ipynb>`__

.. |2| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathTracking/move_to_pose/animation.gif
.. |3| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathTracking/pure_pursuit/animation.gif
.. |4| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathTracking/stanley_controller/animation.gif
.. |5| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathTracking/rear_wheel_feedback/animation.gif
.. |6| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathTracking/lqr_steer_control/animation.gif
.. |7| image:: https://github.com/AtsushiSakai/PythonRobotics/raw/master/PathTracking/lqr_speed_steer_control/animation.gif
