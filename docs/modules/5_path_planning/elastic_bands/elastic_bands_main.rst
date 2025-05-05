Elastic Bands
-------------

This is a path planning with Elastic Bands.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ElasticBands/animation.gif

Code Link
+++++++++++++

.. autoclass:: PathPlanning.ElasticBands.elastic_bands.ElasticBands


Core Concept
~~~~~~~~~~~~
- **Elastic Band**: A dynamically deformable collision-free path initialized by a global planner.
- **Objective**:

  * Shorten and smooth the path.
  * Maximize obstacle clearance.
  * Maintain global path connectivity.

Bubble Representation
~~~~~~~~~~~~~~~~~~~~~~~~
- **Definition**: A local free-space region around configuration :math:`b`:

  .. math::
     B(b) = \{ q: \|q - b\| < \rho(b) \},
  
  where :math:`\rho(b)` is the radius of the bubble.


Force-Based Deformation
~~~~~~~~~~~~~~~~~~~~~~~
The elastic band deforms under artificial forces:

Internal Contraction Force
++++++++++++++++++++++++++
- **Purpose**: Reduces path slack and length.
- **Formula**: For node :math:`b_i`:

  .. math::
     f_c(b_i) = k_c \left( \frac{b_{i-1} - b_i}{\|b_{i-1} - b_i\|} + \frac{b_{i+1} - b_i}{\|b_{i+1} - b_i\|} \right)

  where :math:`k_c` is the contraction gain.

External Repulsion Force
+++++++++++++++++++++++++
- **Purpose**: Pushes the path away from obstacles.
- **Formula**: For node :math:`b_i`:

  .. math::
     f_r(b_i) = \begin{cases} 
     k_r (\rho_0 - \rho(b_i)) \nabla \rho(b_i) & \text{if } \rho(b_i) < \rho_0, \\
     0 & \text{otherwise}.
     \end{cases}

  where :math:`k_r` is the repulsion gain, :math:`\rho_0` is the maximum distance for applying repulsion force, and :math:`\nabla \rho(b_i)` is approximated via finite differences:

  .. math::
     \frac{\partial \rho}{\partial x} \approx \frac{\rho(b_i + h) - \rho(b_i - h)}{2h}.

Dynamic Path Maintenance
~~~~~~~~~~~~~~~~~~~~~~~~~~
1. **Node Update**:
   
   .. math::
      b_i^{\text{new}} = b_i^{\text{old}} + \alpha (f_c + f_r),

   where :math:`\alpha` is a step-size parameter, which often proportional to :math:`\rho(b_i^{\text{old}})`

2. **Overlap Enforcement**:
- Insert new nodes if adjacent nodes are too far apart
- Remove redundant nodes if adjacent nodes are too close

References
+++++++++++++

-  `Elastic Bands: Connecting Path Planning and Control <http://www8.cs.umu.se/research/ifor/dl/Control/elastic%20bands.pdf>`__
