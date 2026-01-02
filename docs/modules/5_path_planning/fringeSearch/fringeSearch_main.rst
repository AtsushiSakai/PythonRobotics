Fringe Search Path Planning
----------------------------

This is a memory-efficient grid-based path planning implementation using Fringe Search.

Fringe Search is an informed search algorithm that combines iterative deepening with A*-style heuristics. It maintains a "fringe" of nodes at the search frontier and iteratively explores paths with increasing f-cost thresholds, providing memory efficiency comparable to iterative deepening while maintaining near-optimal performance.

Code Link
~~~~~~~~~

.. autofunction:: PathPlanning.FringeSearch.fringe_search.main

Algorithm Overview
~~~~~~~~~~~~~~~~~~

Fringe Search operates in iterations with increasing f-cost thresholds:

1. **Initialize**: Start with initial node in fringe, set f-limit to heuristic value
2. **Iterate**: Process nodes in fringe with f-cost ≤ current threshold
3. **Expand**: Generate successor nodes and calculate their f-costs
4. **Defer**: Nodes exceeding threshold are deferred to next iteration
5. **Update**: Increase threshold to minimum deferred f-cost
6. **Repeat**: Continue until goal is found

Animation
~~~~~~~~~

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/FringeSearch/animation.gif

Mathematical Foundation
~~~~~~~~~~~~~~~~~~~~~~~

F-cost Calculation
^^^^^^^^^^^^^^^^^^

The f-cost combines path cost and heuristic:

.. math::

   f(n) = g(n) + h(n)

Where:

- :math:`f(n)` = estimated total cost through node n
- :math:`g(n)` = actual cost from start to node n
- :math:`h(n)` = heuristic estimate from node n to goal

Threshold Update
^^^^^^^^^^^^^^^^

After each iteration, the threshold is updated:

.. math::

   f_{limit}^{new} = \min_{n \in Fringe} f(n) \text{ where } f(n) > f_{limit}^{old}

This ensures monotonic increase in threshold while exploring all nodes within each bound.

Heuristic Function
^^^^^^^^^^^^^^^^^^

Euclidean distance is used as the heuristic:

.. math::

   h(n) = \sqrt{(x_n - x_g)^2 + (y_n - y_g)^2}

Where :math:`(x_n, y_n)` is the current position and :math:`(x_g, y_g)` is the goal position.

Key Features
~~~~~~~~~~~~

**Memory Efficiency**
   - No priority queue (unlike A*)
   - Small fringe list maintained
   - O(b*d) memory complexity where b=branching factor, d=depth

**Cache Management**
   - Tracks visited nodes with (node, f-cost, in_fringe) tuples
   - Efficient lookup for duplicate detection
   - Updates costs when better paths found

**Iterative Deepening**
   - Gradually increases f-cost threshold
   - Avoids exploring high-cost regions prematurely
   - Guarantees optimality (with admissible heuristic)

Advantages
~~~~~~~~~~

- **Low memory usage**: No large priority queue or open list
- **Optimal paths**: Guarantees optimal solution with admissible heuristic
- **Simple implementation**: Easier to understand than A*
- **Cache-friendly**: Better memory locality than A*
- **Performance**: Competitive with A* on grid maps

Disadvantages
~~~~~~~~~~~~~

- **Iteration overhead**: May re-expand nodes across iterations
- **Grid-based**: Best suited for uniform-cost grids
- **Not anytime**: Must complete iteration to get valid path
- **Threshold selection**: Performance sensitive to f-limit updates

Comparison with A*
~~~~~~~~~~~~~~~~~~

+------------------+------------------+------------------+
| Aspect           | A*               | Fringe Search    |
+==================+==================+==================+
| Memory           | O(b^d)           | O(b*d)           |
+------------------+------------------+------------------+
| Data Structure   | Priority Queue   | Simple List      |
+------------------+------------------+------------------+
| Optimality       | Yes              | Yes              |
+------------------+------------------+------------------+
| Speed            | Fast             | Comparable       |
+------------------+------------------+------------------+
| Complexity       | Medium           | Low              |
+------------------+------------------+------------------+

Use Cases
~~~~~~~~~

- **Memory-constrained systems**: Embedded robotics, mobile devices
- **Grid-based games**: Pathfinding in tile-based games
- **Large maps**: When A* priority queue becomes too large
- **Educational**: Simpler to teach than A*

Reference
~~~~~~~~~

- `Fringe Search Paper - Björnsson et al. (2005) <https://webdocs.cs.ualberta.ca/~holte/Publications/fringe.pdf>`__
- `A* search algorithm - Wikipedia <https://en.wikipedia.org/wiki/A*_search_algorithm>`__
- Björnsson, Y.; Enzenberger, M.; Holte, R.; Schaeffer, J. (2005). "Fringe Search: Beating A* at Pathfinding on Game Maps". Proceedings of the National Conference on Artificial Intelligence.
