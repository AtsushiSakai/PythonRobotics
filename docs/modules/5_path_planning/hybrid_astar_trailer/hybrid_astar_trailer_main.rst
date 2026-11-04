Hybrid A* with a Trailer
=============================

A car towing a passive trailer needs more space to turn, and reversing can
increase the angle between the two bodies. This example extends Hybrid A* to
search over both headings, check both vehicle footprints, and arrive with the
trailer aligned to the requested goal.

.. image:: https://raw.githubusercontent.com/AtsushiSakai/PythonRoboticsGifs/e73e6c50d46b8a24d64594a672509bf63fc235d9/PathPlanning/HybridAStarTrailer/animation.gif
   :alt: A tractor and trailer drive around an obstacle and reverse into the goal.

The blue outline is the tractor and the orange outline is the trailer. The cyan
dashed line shows the hitch path; the orange line follows the trailer axle.
Green marks the start, and red marks the goal and its desired body outlines.
Black points are obstacles. The title shows forward/reverse motion and the
current articulation angle.

Kinematic model
---------------

The hitch is at the tractor's rear axle. Let :math:`(x,y)` be its position,
:math:`\theta` the tractor heading, :math:`\phi` the trailer heading,
:math:`L` the tractor wheelbase, and :math:`d` the hitch-to-trailer-axle distance.
With signed speed :math:`v` and tractor steering angle :math:`\delta`,

.. math::

   \dot{x} = v\cos\theta, \qquad
   \dot{y} = v\sin\theta, \qquad
   \dot{\theta} = \frac{v}{L}\tan\delta, \qquad
   \dot{\phi} = \frac{v}{d}\sin(\theta-\phi).

The trailer axle is at
:math:`(x-d\cos\phi,\ y-d\sin\phi)`.
For straight forward travel the trailer tends to align with the tractor;
reversing can amplify the articulation
:math:`\gamma=\operatorname{wrap}(\theta-\phi)`.
The model has one on-axle trailer and no steerable trailer wheels.

Search and goal connection
--------------------------

#. Discretize :math:`(x,y,\theta,\phi)` for the search, while keeping the
   continuous pose at each node. Direction and the previous steering input are
   also retained because they affect transition costs.
#. Expand forward and reverse constant-steering motion primitives. Integrate
   the tractor along a circular arc and the trailer heading with fourth-order
   Runge--Kutta, using signed distance steps.
#. Reject trajectories whose sampled poses collide, exceed the articulation
   limit, or leave the hitch-position search bounds.
#. Try Reeds--Shepp connections for the tractor. Propagate the trailer along
   every candidate and reject connections that leave its final heading outside
   the goal tolerance. The trailer heading is never snapped to the goal.
#. Recover the continuous path through the accepted predecessor segments.

The accumulated cost penalizes distance, reversing, direction changes,
steering, steering changes, and articulation. For one constant-steering segment,

.. math::

   \Delta g = w_{\mathrm{dir}}\ell
       + w_\delta |\delta|\ell
       + w_{\Delta\delta}|\delta-\delta_{\mathrm{prev}}|
       + w_{\mathrm{switch}}\mathbf{1}_{\mathrm{direction\ change}}
       + w_\gamma\int_0^\ell |\gamma(s)|\,ds,

where :math:`\ell` is the positive segment length and
:math:`w_{\mathrm{dir}}` is larger for reverse motion. A weighted Euclidean
hitch-to-goal distance guides the search. This educational implementation uses
finite grid and control resolutions and a bounded number of expansions; it does
not guarantee completeness or the minimum-cost path. It does not perform the
trajectory-smoothing stage of Dolgov et al.

The example uses 2 m position cells, 15 degree heading cells, and motion samples
no farther than 0.2 m apart. Both bodies are rectangles checked against obstacle
points at every motion sample. These are discrete collision checks, not a
continuous swept-volume calculation; obstacle boundaries must be sampled densely.
The articulation limit is 75 degrees. A goal connection must match the tractor
pose within numerical tolerance and the trailer heading within 5 degrees.
The returned heading retains this residual error.

Code
----

.. autofunction:: PathPlanning.HybridAStarTrailer.trailer_hybrid_a_star.hybrid_a_star_planning

.. autofunction:: PathPlanning.HybridAStarTrailer.trailer_hybrid_a_star.move

References
----------

- `Dolgov et al., Practical Search Techniques in Path Planning for Autonomous Driving
  <https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf>`_.
- `LaValle, Planning Algorithms, Section 13.1.2.4: A car pulling trailers
  <https://lavalle.pl/planning/node661.html>`_.
- `Atsushi Sakai's HybridAStarTrailer reference implementation (Julia)
  <https://github.com/yinflight/HybridAStarTrailer>`_.
