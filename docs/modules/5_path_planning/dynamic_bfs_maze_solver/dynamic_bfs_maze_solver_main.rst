Dynamic Maze Solver using Breadth-First Search (BFS)
====================================================

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

This example demonstrates a **dynamic maze-solving algorithm** based on the **Breadth-First Search (BFS)** strategy.
The visualizer dynamically updates a maze in real time while the solver attempts to reach a moving target.

Features
~~~~~~~~

- **Dynamic Maze Solving**: Real-time BFS pathfinding on a grid
- **Moving Target**: The target position changes dynamically during the search
- **Evolving Obstacles**: Obstacles can appear/disappear randomly during execution
- **Visual Feedback**: 
  
  - Black dots represent obstacles
  - Solver position is shown with live updates
  - Path visualization shows the solution trajectory
  - Breadcrumb trail displays the solver's movement history

Sample Animation
~~~~~~~~~~~~~~~~

A sample animation frame from the dynamic maze solver:

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DynamicMazeSolver/animation.gif
   :width: 600
   :align: center

The visualization shows:
   - **Grid Layout**: The maze environment with obstacles (black points)
   - **Target**: Red/green markers indicating the goal position
   - **Path**: The solution path traced by the BFS algorithm
   - **Real-time Updates**: Continuous animation showing the solver in action

Algorithmic Background
----------------------

Breadth-First Search (BFS)
--------------------------

The BFS algorithm is a graph traversal method that explores nodes in layers, guaranteeing the shortest path in an unweighted grid.

.. math::

   M = \{ (i, j) \mid 0 \leq i < R, 0 \leq j < C \}

where R and C denote the number of rows and columns in the grid.

Usage Example
~~~~~~~~~~~~~

To run the dynamic maze solver:

.. code-block:: python

    from PathPlanning.DynamicMazeSolver.dynamic_maze_solver import MazeVisualizer
    
    # Define the maze (0 = free space, 1 = obstacle)
    initial_maze = [
        [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 0, 1, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0, 1, 1, 1, 1, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [1, 1, 1, 1, 0, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]
    
    # Set start and end points
    start_point = (0, 0)
    end_point = (8, 9)
    
    # Create visualizer and run
    visualizer = MazeVisualizer(initial_maze, start_point, end_point)
    visualizer.run()

Expected Output
~~~~~~~~~~~~~~~

When you run the solver, you will see an interactive matplotlib window displaying:

- **Grid with Obstacles**: Black points representing walls and obstacles
- **Start Position**: Marked at the initial coordinates
- **Target Position**: The goal location (changes dynamically)
- **Solver Path**: The trajectory computed by BFS algorithm
- **Animation**: Real-time visualization of the pathfinding process
- **Status Bar**: Shows frame count and current path length

The animation continues until either:
   - The solver reaches the target position (success)
   - The maximum number of frames is reached (500 frames by default)

Code Link
+++++++++

.. autoclass:: PathPlanning.DynamicMazeSolver.dynamic_maze_solver.MazeVisualizer
   :members:
   :undoc-members:
   :show-inheritance:


References
----------

- **Algorithm:** `Breadth-First Search (BFS) <https://en.wikipedia.org/wiki/Breadth-first_search>`_
- **Visualization:** Matplotlib animation
- **Maze Solver:** `AI Maze BFS Example <https://medium.com/@luthfisauqi17_68455/artificial-intelligence-search-problem-solve-maze-using-breadth-first-search-bfs-algorithm-255139c6e1a3>`_
