Dynamic Maze Solver using Breadth-First Search (BFS)
====================================================

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

This example demonstrates a **dynamic maze-solving algorithm** based on the **Breadth-First Search (BFS)** strategy.
The visualizer dynamically updates a maze in real time while the solver attempts to reach a moving target.

A sample animation frame:

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/dynamic_maze_solver/animation.gif

Algorithmic Background
----------------------

### Breadth-First Search (BFS)

The BFS algorithm is a graph traversal method that explores nodes in layers, guaranteeing the shortest path in an unweighted grid.

.. math::

   M = \{ (i, j) \mid 0 \leq i < R, 0 \leq j < C \}

...

Code Link
+++++++++

.. automodule:: PathPlanning.BreadthFirstSearch.dynamic_maze_solver


References
----------

- **Algorithm:** `Breadth-First Search (BFS) <https://en.wikipedia.org/wiki/Breadth-first_search>`_
- **Visualization:** Matplotlib animation
- **Maze Solver:** `AI Maze BFS Example <https://medium.com/@luthfisauqi17_68455/artificial-intelligence-search-problem-solve-maze-using-breadth-first-search-bfs-algorithm-255139c6e1a3>`_
