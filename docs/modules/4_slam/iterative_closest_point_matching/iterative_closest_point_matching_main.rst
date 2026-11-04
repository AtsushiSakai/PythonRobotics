.. _iterative-closest-point-(icp)-matching:

Iterative Closest Point (ICP) Matching
--------------------------------------

This is a 2D and 3D ICP matching example with singular value decomposition.

It can calculate a rotation matrix and a translation vector between
points to points.

Accumulating rigid motions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

At each iteration, nearest-neighbor correspondences are used to estimate a
rotation :math:`R_k` and translation :math:`t_k` that align the current cloud
with the previous cloud. For column-vector points, the update is

.. math::

   p_{k+1} = R_k p_k + t_k,
   \qquad
   H_k = \begin{bmatrix} R_k & t_k \\ 0 & 1 \end{bmatrix}.

The accumulated transform must apply these increments in the same order as
the point updates:

.. math::

   H_{\mathrm{acc}, k+1} = H_k H_{\mathrm{acc}, k},
   \qquad H_{\mathrm{acc}, 0} = I.

Rigid motions generally do not commute. Reversing the multiplication order
can therefore return a transform that does not align the original input
cloud, even when the internal residual has reached machine precision. The returned
rotation and translation are the blocks of the accumulated transform and
map the original current cloud into the previous frame.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/SLAM/iterative_closest_point/animation.gif

Code Link
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. autofunction:: SLAM.ICPMatching.icp_matching.icp_matching


References
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- `Introduction to Mobile Robotics: Iterative Closest Point Algorithm <https://cs.gmu.edu/~kosecka/cs685/cs685-icp.pdf>`_
