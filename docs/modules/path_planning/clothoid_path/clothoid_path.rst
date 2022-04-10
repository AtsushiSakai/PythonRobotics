.. _clothoid-path-planning:

Clothoid path planning
--------------------------

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ClothoidPath/animation1.gif
.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ClothoidPath/animation2.gif
.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ClothoidPath/animation3.gif

This is a clothoid path planning sample code.

This can interpolate two 2D pose (x, y, yaw) with a clothoid path.
This means its curvature is linearly continuous.
In other words, this is G1 Hermite interpolation with a single clothoid segment.

This path planning algorithm as follows:

Step1: Solve g function
~~~~~~~~~~~~~~~~~~~~~~~

Step2: Calculate path parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


.. math::

        L=\frac{\sqrt{\Delta x^{2}+\Delta y^{2}}}{X\left(2 A, \delta-A, \phi_{0}\right)}

Step3: Construct a path with Fresnel integral
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The final clothoid path can be calculated with the path parameters and Fresnel integrals.

\begin{aligned}
&x(s)=x_{0}+\int_{0}^{s} \cos \left(\frac{1}{2} \kappa^{\prime} \tau^{2}+\kappa \tau+\vartheta_{0}\right) \mathrm{d} \tau \\
&y(s)=y_{0}+\int_{0}^{s} \sin \left(\frac{1}{2} \kappa^{\prime} \tau^{2}+\kappa \tau+\vartheta_{0}\right) \mathrm{d} \tau
\end{aligned}


References
~~~~~~~~~~

-  `Fast and accurate G1 fitting of clothoid curves <https://www.researchgate.net/publication/237062806>`__
