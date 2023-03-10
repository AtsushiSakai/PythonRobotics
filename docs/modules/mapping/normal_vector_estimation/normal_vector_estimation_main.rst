Normal vector estimation
-------------------------


Normal vector calculation of a 3D triangle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A 3D point is as a vector:

.. math:: p = [x, y, z]

When there are 3 points in 3D space, :math:`p_1, p_2, p_3`,

we can calculate a normal vector n of a 3D triangle which is consisted of the points.

.. math:: n = \frac{v1 \times v2}{|v1 \times v2|}

where

.. math:: v1 = p2 - p1

.. math:: v2 = p3 - p1



Normal vector estimation with RANdam SAmpling Consensus(RANSAC)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


