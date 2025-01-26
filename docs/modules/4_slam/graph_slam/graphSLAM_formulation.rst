.. _Graph SLAM Formulation:

Graph SLAM Formulation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Author Jeff Irion

Problem Formulation
^^^^^^^^^^^^^^^^^^^

Let a robot’s trajectory through its environment be represented by a
sequence of :math:`N` poses:
:math:`\mathbf{p}_1, \mathbf{p}_2, \ldots, \mathbf{p}_N`. Each pose lies
on a manifold: :math:`\mathbf{p}_i \in \mathcal{M}`. Simple examples of
manifolds used in Graph SLAM include 1-D, 2-D, and 3-D space, i.e.,
:math:`\mathbb{R}`, :math:`\mathbb{R}^2`, and :math:`\mathbb{R}^3`.
These environments are *rectilinear*, meaning that there is no concept
of orientation. By contrast, in :math:`SE(2)` problem settings a robot’s
pose consists of its location in :math:`\mathbb{R}^2` and its
orientation :math:`\theta`. Similarly, in :math:`SE(3)` a robot’s pose
consists of its location in :math:`\mathbb{R}^3` and its orientation,
which can be represented via Euler angles, quaternions, or :math:`SO(3)`
rotation matrices.

As the robot explores its environment, it collects a set of :math:`M`
measurements :math:`\mathcal{Z} = \{\mathbf{z}_j\}`. Examples of such
measurements include odometry, GPS, and IMU data. Given a set of poses
:math:`\mathbf{p}_1, \ldots, \mathbf{p}_N`, we can compute the estimated
measurement
:math:`\hat{\mathbf{z}}_j(\mathbf{p}_1, \ldots, \mathbf{p}_N)`. We can
then compute the *residual*
:math:`\mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j)` for measurement
:math:`j`. The formula for the residual depends on the type of
measurement. As an example, let :math:`\mathbf{z}_1` be an odometry
measurement that was collected when the robot traveled from
:math:`\mathbf{p}_1` to :math:`\mathbf{p}_2`. The expected measurement
and the residual are computed as

.. math::

   \begin{aligned}
       \hat{\mathbf{z}}_1(\mathbf{p}_1, \mathbf{p}_2) &= \mathbf{p}_2 \ominus \mathbf{p}_1 \\
       \mathbf{e}_1(\mathbf{z}_1, \hat{\mathbf{z}}_1) &= \mathbf{z}_1 \ominus \hat{\mathbf{z}}_1 = \mathbf{z}_1 \ominus (\mathbf{p}_2 \ominus \mathbf{p}_1),\end{aligned}

where the :math:`\ominus` operator indicates inverse pose composition.
We model measurement :math:`\mathbf{z}_j` as having independent Gaussian
noise with zero mean and covariance matrix :math:`\Omega_j^{-1}`; we
refer to :math:`\Omega_j` as the *information matrix* for measurement
:math:`j`. That is,

.. math::
    p(\mathbf{z}_j \ | \ \mathbf{p}_1, \ldots, \mathbf{p}_N) = \eta_j \exp (-\mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j))^{\mathsf{T}}\Omega_j \mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j)
    :label: infomat

where :math:`\eta_j` is the normalization constant.

The objective of Graph SLAM is to find the maximum likelihood set of
poses given the measurements :math:`\mathcal{Z} = \{\mathbf{z}_j\}`; in
other words, we want to find

.. math:: \mathop{\mathrm{arg\,max}}_{\mathbf{p}_1, \ldots, \mathbf{p}_N} \ p(\mathbf{p}_1, \ldots, \mathbf{p}_N \ | \ \mathcal{Z})

Using Bayes’ rule, we can write this probability as

.. math::
    \begin{aligned}
       p(\mathbf{p}_1, \ldots, \mathbf{p}_N \ | \ \mathcal{Z}) &= \frac{p( \mathcal{Z} \ | \ \mathbf{p}_1, \ldots, \mathbf{p}_N) p(\mathbf{p}_1, \ldots, \mathbf{p}_N) }{ p(\mathcal{Z}) } \notag \\
       &\propto p( \mathcal{Z} \ | \ \mathbf{p}_1, \ldots, \mathbf{p}_N)
    \end{aligned}
    :label: bayes

since :math:`p(\mathcal{Z})` is a constant (albeit, an unknown constant)
and we assume that :math:`p(\mathbf{p}_1, \ldots, \mathbf{p}_N)` is
uniformly distributed. Therefore, we
can use Eq. :eq:`infomat` and and Eq. :eq:`bayes` to simplify the Graph SLAM
optimization as follows:

.. math::

   \begin{aligned}
       \mathop{\mathrm{arg\,max}}_{\mathbf{p}_1, \ldots, \mathbf{p}_N} \ p(\mathbf{p}_1, \ldots, \mathbf{p}_N \ | \ \mathcal{Z}) &= \mathop{\mathrm{arg\,max}}_{\mathbf{p}_1, \ldots, \mathbf{p}_N} \ p( \mathcal{Z} \ | \ \mathbf{p}_1, \ldots, \mathbf{p}_N) \\
       &= \mathop{\mathrm{arg\,max}}_{\mathbf{p}_1, \ldots, \mathbf{p}_N} \prod_{j=1}^M p(\mathbf{z}_j \ | \ \mathbf{p}_1, \ldots, \mathbf{p}_N) \\
       &= \mathop{\mathrm{arg\,max}}_{\mathbf{p}_1, \ldots, \mathbf{p}_N} \prod_{j=1}^M \exp \left( -(\mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j))^{\scriptstyle{\mathsf{T}}}\Omega_j \mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j) \right) \\
       &= \mathop{\mathrm{arg\,min}}_{\mathbf{p}_1, \ldots, \mathbf{p}_N} \sum_{j=1}^M (\mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j))^{\scriptstyle{\mathsf{T}}}\Omega_j \mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j).\end{aligned}

We define

.. math:: \chi^2 := \sum_{j=1}^M (\mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j))^{\scriptstyle{\mathsf{T}}}\Omega_j \mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j),

and this is what we seek to minimize.

Dimensionality and Pose Representation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before proceeding further, it is helpful to discuss the dimensionality
of the problem. We have:

-  A set of :math:`N` poses
   :math:`\mathbf{p}_1, \mathbf{p}_2, \ldots, \mathbf{p}_N`, where each
   pose lies on the manifold :math:`\mathcal{M}`

   -  Each pose :math:`\mathbf{p}_i` is represented as a vector in (a
      subset of) :math:`\mathbb{R}^d`. For example:

      -  An :math:`SE(2)` pose is typically represented as
         :math:`(x, y, \theta)`, and thus :math:`d = 3`.

      -  An :math:`SE(3)` pose is typically represented as
         :math:`(x, y, z, q_x, q_y, q_z, q_w)`, where :math:`(x, y, z)`
         is a point in :math:`\mathbb{R}^3` and
         :math:`(q_x, q_y, q_z, q_w)` is a *quaternion*, and so
         :math:`d = 7`. For more information about :math:`SE(3)`
         parameterization and pose transformations, see
         [blanco2010tutorial]_.

   -  We also need to be able to represent each pose compactly as a
      vector in (a subset of) :math:`\mathbb{R}^c`.

      -  Since an :math:`SE(2)` pose has three degrees of freedom, the
         :math:`(x, y, \theta)` representation is again sufficient and
         :math:`c=3`.

      -  An :math:`SE(3)` pose only has six degrees of freedom, and we
         can represent it compactly as :math:`(x, y, z, q_x, q_y, q_z)`,
         and thus :math:`c=6`.

   -  We use the :math:`\boxplus` operator to indicate pose composition
      when one or both of the poses are represented compactly. The
      output can be a pose in :math:`\mathcal{M}` or a vector in
      :math:`\mathbb{R}^c`, as required by context.

-  A set of :math:`M` measurements
   :math:`\mathcal{Z} = \{\mathbf{z}_1, \mathbf{z}_2, \ldots, \mathbf{z}_M\}`

   -  Each measurement’s dimensionality can be unique, and we will use
      :math:`\bullet` to denote a “wildcard” variable.

   -  Measurement :math:`\mathbf{z}_j \in \mathbb{R}^\bullet` has an
      associated information matrix
      :math:`\Omega_j \in \mathbb{R}^{\bullet \times \bullet}` and
      residual function
      :math:`\mathbf{e}_j(\mathbf{z}_j, \hat{\mathbf{z}}_j) = \mathbf{e}_j(\mathbf{z}_j, \mathbf{p}_1, \ldots, \mathbf{p}_N) \in \mathbb{R}^\bullet`.

   -  A measurement could, in theory, constrain anywhere from 1 pose to
      all :math:`N` poses. In practice, each measurement usually
      constrains only 1 or 2 poses.

Graph SLAM Algorithm
^^^^^^^^^^^^^^^^^^^^

The “Graph” in Graph SLAM refers to the fact that we view the problem as
a graph. The graph has a set :math:`\mathcal{V}` of :math:`N` vertices,
where each vertex :math:`v_i` has an associated pose
:math:`\mathbf{p}_i`. Similarly, the graph has a set :math:`\mathcal{E}`
of :math:`M` edges, where each edge :math:`e_j` has an associated
measurement :math:`\mathbf{z}_j`. In practice, the edges in this graph
are either unary (i.e., a loop) or binary. (Note: :math:`e_j` refers to
the edge in the graph associated with measurement :math:`\mathbf{z}_j`,
whereas :math:`\mathbf{e}_j` refers to the residual function associated
with :math:`\mathbf{z}_j`.) For more information about the Graph SLAM
algorithm, see [grisetti2010tutorial]_.

We want to optimize

.. math:: \chi^2 = \sum_{e_j \in \mathcal{E}} \mathbf{e}_j^{\scriptstyle{\mathsf{T}}}\Omega_j \mathbf{e}_j.

Let :math:`\mathbf{x}_i \in \mathbb{R}^c` be the compact representation
of pose :math:`\mathbf{p}_i \in \mathcal{M}`, and let

.. math:: \mathbf{x} := \begin{bmatrix} \mathbf{x}_1 \\ \mathbf{x}_2 \\ \vdots \\ \mathbf{x}_N \end{bmatrix} \in \mathbb{R}^{cN}

We will solve this optimization problem iteratively. Let

.. math:: \mathbf{x}^{k+1} := \mathbf{x}^k \boxplus \Delta \mathbf{x}^k = \begin{bmatrix} \mathbf{x}_1 \boxplus \Delta \mathbf{x}_1 \\ \mathbf{x}_2 \boxplus \Delta \mathbf{x}_2 \\ \vdots \\ \mathbf{x}_N \boxplus \Delta \mathbf{x}_2 \end{bmatrix}
    :label: update

The :math:`\chi^2` error at iteration :math:`k+1` is

.. math:: \chi_{k+1}^2 = \sum_{e_j \in \mathcal{E}} \underbrace{\left[ \mathbf{e}_j(\mathbf{x}^{k+1}) \right]^{\scriptstyle{\mathsf{T}}}}_{1 \times \bullet} \underbrace{\Omega_j}_{\bullet \times \bullet} \underbrace{\mathbf{e}_j(\mathbf{x}^{k+1})}_{\bullet \times 1}.
    :label: chisq_at_kplusone

We will linearize the residuals as:

.. math::
    \begin{aligned}
        \mathbf{e}_j(\mathbf{x}^{k+1}) &= \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k) \\
        &\approx \mathbf{e}_j(\mathbf{x}^{k}) + \frac{\partial}{\partial \Delta \mathbf{x}^k} \left[ \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k) \right] \Delta \mathbf{x}^k \\
        &= \mathbf{e}_j(\mathbf{x}^{k}) + \left( \left. \frac{\partial \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)} \right|_{\Delta \mathbf{x}^k = \mathbf{0}} \right) \frac{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial \Delta \mathbf{x}^k} \Delta \mathbf{x}^k.
    \end{aligned}
    :label: linearization

Plugging :eq:`linearization` into :eq:`chisq_at_kplusone`, we get:

.. math::

   \begin{aligned}
       \chi_{k+1}^2 &\approx \ \ \ \ \ \sum_{e_j \in \mathcal{E}} \underbrace{[ \mathbf{e}_j(\mathbf{x}^k)]^{\scriptstyle{\mathsf{T}}}}_{1 \times \bullet} \underbrace{\Omega_j}_{\bullet \times \bullet} \underbrace{\mathbf{e}_j(\mathbf{x}^k)}_{\bullet \times 1} \notag \\
       &\hphantom{\approx} \ \ \ + \sum_{e_j \in \mathcal{E}} \underbrace{[ \mathbf{e}_j(\mathbf{x^k}) ]^{\scriptstyle{\mathsf{T}}}}_{1 \times \bullet} \underbrace{\Omega_j}_{\bullet \times \bullet} \underbrace{\left( \left. \frac{\partial \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)} \right|_{\Delta \mathbf{x}^k = \mathbf{0}} \right)}_{\bullet \times dN} \underbrace{\frac{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial \Delta \mathbf{x}^k}}_{dN \times cN} \underbrace{\Delta \mathbf{x}^k}_{cN \times 1} \notag \\
       &\hphantom{\approx} \ \ \ + \sum_{e_j \in \mathcal{E}} \underbrace{(\Delta \mathbf{x}^k)^{\scriptstyle{\mathsf{T}}}}_{1 \times cN} \underbrace{ \left( \frac{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial \Delta \mathbf{x}^k} \right)^{\scriptstyle{\mathsf{T}}}}_{cN \times dN} \underbrace{\left( \left. \frac{\partial \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)} \right|_{\Delta \mathbf{x}^k = \mathbf{0}} \right)^{\scriptstyle{\mathsf{T}}}}_{dN \times \bullet} \underbrace{\Omega_j}_{\bullet \times \bullet} \underbrace{\left( \left. \frac{\partial \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)} \right|_{\Delta \mathbf{x}^k = \mathbf{0}} \right)}_{\bullet \times dN} \underbrace{\frac{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial \Delta \mathbf{x}^k}}_{dN \times cN} \underbrace{\Delta \mathbf{x}^k}_{cN \times 1} \notag \\
       &= \chi_k^2 + 2 \mathbf{b}^{\scriptstyle{\mathsf{T}}}\Delta \mathbf{x}^k + (\Delta \mathbf{x}^k)^{\scriptstyle{\mathsf{T}}}H \Delta \mathbf{x}^k,  \notag\end{aligned}

where

.. math::

   \begin{aligned}
       \mathbf{b}^{\scriptstyle{\mathsf{T}}}&= \sum_{e_j \in \mathcal{E}} \underbrace{[ \mathbf{e}_j(\mathbf{x^k}) ]^{\scriptstyle{\mathsf{T}}}}_{1 \times \bullet} \underbrace{\Omega_j}_{\bullet \times \bullet} \underbrace{\left( \left. \frac{\partial \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)} \right|_{\Delta \mathbf{x}^k = \mathbf{0}} \right)}_{\bullet \times dN} \underbrace{\frac{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial \Delta \mathbf{x}^k}}_{dN \times cN} \\
       H &= \sum_{e_j \in \mathcal{E}} \underbrace{ \left( \frac{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial \Delta \mathbf{x}^k} \right)^{\scriptstyle{\mathsf{T}}}}_{cN \times dN} \underbrace{\left( \left. \frac{\partial \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)} \right|_{\Delta \mathbf{x}^k = \mathbf{0}} \right)^{\scriptstyle{\mathsf{T}}}}_{dN \times \bullet} \underbrace{\Omega_j}_{\bullet \times \bullet} \underbrace{\left( \left. \frac{\partial \mathbf{e}_j(\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)} \right|_{\Delta \mathbf{x}^k = \mathbf{0}} \right)}_{\bullet \times dN} \underbrace{\frac{\partial (\mathbf{x}^k \boxplus \Delta \mathbf{x}^k)}{\partial \Delta \mathbf{x}^k}}_{dN \times cN}.\end{aligned}

Using this notation, we obtain the optimal update as

.. math:: \Delta \mathbf{x}^k = -H^{-1} \mathbf{b}.  \label{eq:deltax}

We apply this update to the poses via :eq:`update` and repeat until convergence.


.. [blanco2010tutorial] Blanco, J.-L.A tutorial onSE(3) transformation parameterization and on-manifold optimization.University of Malaga, Tech. Rep 3(2010)
.. [grisetti2010tutorial] Grisetti, G., Kummerle, R., Stachniss, C., and Burgard, W.A tutorial on graph-based SLAM.IEEE Intelligent Transportation Systems Magazine 2, 4 (2010), 31–43.

