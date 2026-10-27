Coverage path planner
---------------------

Grid based sweep
~~~~~~~~~~~~~~~~

This is a 2D grid based sweep coverage path planner simulation:

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/GridBasedSweepCPP/animation.gif

The underlying occupancy grid has finite boundaries. Cell indices range from
zero to one less than the number of columns or rows. Cells outside either
axis are treated as occupied by the sweep search, and obstacle expansion is
clipped at the grid boundary rather than wrapping into another row.

Code Link
+++++++++++++

.. autofunction:: PathPlanning.GridBasedSweepCPP.grid_based_sweep_coverage_path_planner.planning

Spiral Spanning Tree
~~~~~~~~~~~~~~~~~~~~

This is a 2D grid based spiral spanning tree coverage path planner simulation:

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/SpiralSpanningTreeCPP/animation1.gif
.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/SpiralSpanningTreeCPP/animation2.gif
.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/SpiralSpanningTreeCPP/animation3.gif

Code Link
+++++++++++++

.. autofunction:: PathPlanning.SpiralSpanningTreeCPP.spiral_spanning_tree_coverage_path_planner.main

Reference
+++++++++++++

- `Spiral-STC: An On-Line Coverage Algorithm of Grid Environments by a Mobile Robot <https://ieeexplore.ieee.org/abstract/document/1013479>`_


Wavefront path
~~~~~~~~~~~~~~

This is a 2D grid based wavefront coverage path planner simulation:

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/WavefrontCPP/animation1.gif
.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/WavefrontCPP/animation2.gif
.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/WavefrontCPP/animation3.gif

Code Link
+++++++++++++

.. autofunction:: PathPlanning.WavefrontCPP.wavefront_coverage_path_planner.wavefront

Reference
+++++++++++++

- `Planning paths of complete coverage of an unstructured environment by a mobile robot <https://pinkwink.kr/attachment/cfile3.uf@1354654A4E8945BD13FE77.pdf>`_


