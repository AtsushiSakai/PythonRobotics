Flocking
========

Olfati-Saber's Algorithm 2 coordinates double-integrator agents in free space:
:math:`\dot q_i=p_i,\ \dot p_i=u_i`.
Nearby agents adjust their spacing and velocities while tracking a shared,
constant-velocity reference :math:`(q_r,p_r)`.

.. image:: https://raw.githubusercontent.com/AtsushiSakai/PythonRoboticsGifs/ca4eb94b0876aa666e44b2125bcce450f7594f30/PathPlanning/Flocking/animation.gif
   :alt: Twenty-five agents forming a flock and following a moving reference

Local interactions
------------------

For :math:`\epsilon>0`, use the smooth distance and its gradient:

.. math::

   \|z\|_\sigma=\frac{\sqrt{1+\epsilon\|z\|^2}-1}{\epsilon},
   \qquad n_{ij}=\frac{q_j-q_i}{\sqrt{1+\epsilon\|q_j-q_i\|^2}}.

The adjacency weight vanishes beyond interaction range :math:`r`:

.. math::

   \rho_h(s)=\begin{cases}
     1 & 0\le s<h,\\
     \frac{1+\cos(\pi(s-h)/(1-h))}{2} & h\le s\le1,\\
     0 & \text{otherwise},
   \end{cases}
   \qquad a_{ij}=\rho_h(\|q_j-q_i\|_\sigma/\|r\|_\sigma),
   \quad a_{ii}=0.

With desired distance :math:`d<r`, this example uses the symmetric choice
:math:`a=b=5` of the paper's action function (equation 15):

.. math::

   \phi_\alpha(s)=5\rho_h(s/\|r\|_\sigma)
     \frac{s-\|d\|_\sigma}{\sqrt{1+(s-\|d\|_\sigma)^2}}.

The sign gives repulsion below :math:`d` and attraction above :math:`d`,
within range. Algorithm 2 (equation 24) combines this with alignment and
linear navigation:

.. math::

   u_i=\underbrace{\sum_{j\ne i}\phi_\alpha(\|q_j-q_i\|_\sigma)n_{ij}}_{\text{spacing}}
       +\underbrace{\sum_{j\ne i}a_{ij}(p_j-p_i)}_{\text{alignment}}
       -\underbrace{c_1(q_i-q_r)+c_2(p_i-p_r)}_{\text{navigation}}.

Simulation
----------

The example starts 25 agents on a perturbed grid, with random velocities.
Parameters are :math:`d=5`, :math:`r=6`, :math:`\epsilon=0.1`,
:math:`h=0.2`, :math:`c_1=0.1`, and :math:`c_2=2\sqrt{c_1}`.
Accelerations are held over each :math:`0.02` s step:

.. math::

   q_i^{k+1}=q_i^k+\Delta t\,p_i^k+\tfrac12\Delta t^2u_i^k,
   \qquad p_i^{k+1}=p_i^k+\Delta t\,u_i^k.

Arrows show velocities; edges show neighbors. The second panel measures
RMS velocity error relative to the moving reference.

These are point agents without obstacles or actuator limits. Finite-step
simulation does not guarantee collision avoidance for arbitrary initial
conditions; exactly coincident agents have zero separation gradient.

Code
----

.. autofunction:: PathPlanning.Flocking.flocking.main

.. autofunction:: PathPlanning.Flocking.flocking.flocking_control

.. autofunction:: PathPlanning.Flocking.flocking.simulate

Reference
---------

R. Olfati-Saber, `Flocking for Multi-Agent Dynamic Systems: Algorithms and Theory
<https://doi.org/10.1109/TAC.2005.864190>`__, IEEE Transactions on Automatic
Control, 51(3), 401–420, 2006.
`Author manuscript <https://hal.elte.hu/~vicsek/downloads/papers/flocking_tac06-engineering.pdf>`__.
