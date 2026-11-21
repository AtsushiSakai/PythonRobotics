Sequential Single-Item Auction
------------------------------

A sequential single-item (SSI) auction is a simple and widely used method for
**multi-robot task allocation**. It decides which robot of a team visits
which target, and in which order, so that a team objective is kept small.
A typical use case is a fleet of drones that must visit a set of inspection
or delivery points.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/MissionPlanning/SequentialSingleItemAuction/animation.gif

The animation first shows the MiniSum auction and then the MiniMax auction on
the same map. Triangles are robot start positions, gray crosses are targets
that are not assigned yet, and colored lines are each robot's current route.
Routes follow A* paths around the walls (black dots).

Problem
~~~~~~~

Given robots with start positions :math:`s_1, \dots, s_m` and targets
:math:`t_1, \dots, t_n`, find an ordered route :math:`R_i` for every robot such
that every target appears in exactly one route. The cost of a route is the
length of the open path that starts at the robot and visits its targets in
order:

.. math::

   c(R_i) = d(s_i, r_1) + \sum_{k=1}^{|R_i|-1} d(r_k, r_{k+1})

where :math:`d(a, b)` is the travel distance between two points. In this
example, :math:`d` is the length of an A* path on an occupancy grid, so the
costs respect obstacles.

Two common team objectives are:

- **MiniSum**: minimize the total travel distance
  :math:`\sum_i c(R_i)`, which corresponds to energy usage.
- **MiniMax**: minimize the longest route :math:`\max_i c(R_i)`, which
  corresponds to the mission completion time.

Finding the optimal allocation is NP-hard (it generalizes the traveling
salesman problem), so practical systems use fast approximations such as
auctions.

Algorithm
~~~~~~~~~

The auction assigns one target per round until every target is assigned:

1. Every robot computes a bid for every unassigned target.
   To do this, it tries inserting the target at every position of its current
   route and keeps the cheapest resulting route cost :math:`c(R_i \oplus t)`.
2. The bid depends on the team objective:

   - MiniSum: :math:`b_i(t) = c(R_i \oplus t) - c(R_i)`, the marginal cost.
   - MiniMax: :math:`b_i(t) = c(R_i \oplus t)`, the robot's new route cost.

3. The single lowest bid over all robots and targets wins. That target is
   inserted into the winning robot's route at the chosen position.

With :math:`m` robots and :math:`n` targets, each round evaluates at most
:math:`m \cdot n` bids, so the auction is fast even for large teams. The
travel distances are cached, so each A* path is computed only once.

Results
~~~~~~~

In the example scenario, the two objectives produce clearly different
allocations:

.. list-table::
   :header-rows: 1

   * - Objective
     - Total distance
     - Longest route
     - Targets per robot
   * - MiniSum
     - 136 m
     - 114 m
     - 0, 9, 1
   * - MiniMax
     - 195 m
     - 76 m
     - 1, 7, 2

MiniSum leaves robot 0 idle because the robot on the right can collect the
cluster of targets most cheaply. MiniMax spends more total distance so that
the work is shared and the mission finishes sooner.

Comparison with related methods
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- The **Hungarian algorithm** finds an optimal assignment when each robot
  takes exactly one task. SSI auctions allow each robot to take several tasks
  and account for the order in which it visits them.
- **Parallel single-item auctions** let every robot bid on all targets once,
  ignoring how targets interact within a route. SSI auctions re-bid after
  every assignment, so they account for those interactions.
- **Combinatorial auctions** let robots bid on bundles of targets and can
  find better allocations, but the number of bundles grows exponentially.
  SSI auctions keep the communication and computation small while capturing
  much of the benefit.

Because the bids are computed by each robot from its own route, the auction
can also run in a distributed way, with an auctioneer that only compares bids.

Code Link
~~~~~~~~~

.. autofunction:: MissionPlanning.SequentialSingleItemAuction.sequential_single_item_auction.sequential_single_item_auction

.. autofunction:: MissionPlanning.SequentialSingleItemAuction.sequential_single_item_auction.best_insertion

.. autofunction:: MissionPlanning.SequentialSingleItemAuction.sequential_single_item_auction.calc_bid

References
~~~~~~~~~~

- M. G. Lagoudakis et al.,
  `Auction-Based Multi-Robot Routing <https://www.roboticsproceedings.org/rss01/p45.pdf>`_,
  Robotics: Science and Systems, 2005.
- S. Koenig et al., "The Power of Sequential Single-Item Auctions for Agent
  Coordination", AAAI, 2006.
