.. _gaussian_grid_map:

Gaussian grid map
-----------------

This is a 2D Gaussian grid mapping example.

A cell at distance *d* from the nearest measurement receives the Gaussian
upper-tail probability. It is 0.5 at the measurement and decreases as the
distance grows relative to the specified standard deviation.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/gaussian_grid_map/animation.gif

Code Link
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. autofunction:: Mapping.gaussian_grid_map.gaussian_grid_map.generate_gaussian_grid_map

