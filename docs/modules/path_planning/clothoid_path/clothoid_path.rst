.. _clothoid-path-planning:

Clothoid path planning
--------------------------

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ClothoidPath/animation1.gif
.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ClothoidPath/animation2.gif
.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ClothoidPath/animation3.gif

This is a clothoid path planning sample code.

This can interpolate two 2D pose (x, y, yaw) with a clothoid path,
which its curvature is linearly continuous.
In other words, this is G1 Hermite interpolation with a single clothoid segment.

This path planning algorithm as follows:

Step1: Solve g function
~~~~~~~~~~~~~~~~~~~~~~~

Solve the g(A) function with a nonlinear optimization solver.

.. math::

    g(A):=Y(2A, \delta-A, \phi_{s})

Where

* :math:`\delta`: the orientation difference between start and goal pose.
* :math:`\phi_{s}`: the orientation of the start pose.
* :math:`Y`: :math:`Y(a, b, c)=\int_{0}^{1} \sin \left(\frac{a}{2} \tau^{2}+b \tau+c\right) d \tau`


Step2: Calculate path parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We can calculate these path parameters using :math:`A`,

:math:`L`: path length

.. math::

        L=\frac{R}{X\left(2 A, \delta-A, \phi_{s}\right)}

where

* :math:`R`: the distance between start and goal pose
* :math:`X`: :math:`X(a, b, c)=\int_{0}^{1} \cos \left(\frac{a}{2} \tau^{2}+b \tau+c\right) d \tau`


- :math:`\kappa`: curvature

.. math::

        \kappa=(\delta-A) / L


- :math:`\kappa'`: curvature rate

.. math::

        \kappa^{\prime}=2 A / L^{2}


Step3: Construct a path with Fresnel integral
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The final clothoid path can be calculated with the path parameters and Fresnel integrals.

.. math::
        \begin{aligned}
        &x(s)=x_{0}+\int_{0}^{s} \cos \left(\frac{1}{2} \kappa^{\prime} \tau^{2}+\kappa \tau+\vartheta_{0}\right) \mathrm{d} \tau \\
        &y(s)=y_{0}+\int_{0}^{s} \sin \left(\frac{1}{2} \kappa^{\prime} \tau^{2}+\kappa \tau+\vartheta_{0}\right) \mathrm{d} \tau
        \end{aligned}


References
~~~~~~~~~~

-  `Fast and accurate G1 fitting of clothoid curves <https://www.researchgate.net/publication/237062806>`__
