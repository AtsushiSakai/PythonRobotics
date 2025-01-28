.. _`What is PythonRobotics?`:

What is PythonRobotics?
------------------------

This is an Open Source Software (OSS) project: PythonRobotics, which is a Python code collection of robotics algorithms.
These codes are developed under `MIT license`_ and on `GitHub`_.

This project has three main philosophies below:

1. Easy to understand each algorithm's basic idea.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The goal is for beginners in robotics to understand the basic ideas behind each algorithm.

`Python`_ programming language is adopted in this project to achieve the goal.
Python is a high-level programming language that is easy to read and write.
If the code is not easy to read, it would be difficult to achieve our goal of
allowing beginners to understand the algorithms.
Python has great libraries for matrix operation, mathematical and scientific operation,
and visualization, which makes code more readable because such operations
don’t need to be re-implemented.
Having the core Python packages allows the user to focus on the algorithms,
rather than the implementations.

PythonRobotics provides not only the code but also intuitive animations that
visually demonstrate the process and behavior of each algorithm over time.
This is an example of an animation gif file that shows the process of the
path planning in a highway scenario for autonomous vehicle.

.. image:: https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/FrenetOptimalTrajectory/high_speed_and_velocity_keeping_frenet_path.gif

This animation is a gif file and is created using the `Matplotlib`_ library.
All animaion gif files are stored in the `PythonRoboticsGifs`_ repository and
all animation movies are uploaded to this `YouTube channel`_.

PythonRobotics also provides a textbook that explains the basic ideas of each algorithm.

.. _`Python`: https://www.python.org/
.. _`PythonRoboticsGifs`: https://github.com/AtsushiSakai/PythonRoboticsGifs
.. _`YouTube channel`: https://youtube.com/playlist?list=PL12URV8HFpCozuz0SDxke6b2ae5UZvIwa&si=AH2fNPPYufPtK20S


2. Widely used and practical algorithms are selected.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The second philosophy is that implemented algorithms have to be practical
and widely used in both academia and industry.
We believe learning these algorithms will be useful in many applications.
For example, Kalman filters and particle filter for localization,
grid mapping for mapping,
dynamic programming based approaches and sampling based approaches for path planning,
and optimal control based approach for path tracking.
These algorithms are implemented in this project.

3. Minimum dependency.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each sample code is written in Python3 and only depends on some standard
modules for readability and ease of use.


.. _GitHub: https://github.com/AtsushiSakai/PythonRobotics
.. _`MIT license`: https://github.com/AtsushiSakai/PythonRobotics/blob/master/LICENSE


See this paper for more details:

- PythonRobotics: a Python code collection of robotics algorithms: https://arxiv.org/abs/1808.10703

.. _`Requirements`:

Requirements
============

-  `Python 3.12.x`_
-  `NumPy`_
-  `SciPy`_
-  `Matplotlib`_
-  `cvxpy`_

For development:

-  `pytest`_ (for unit tests)
-  `pytest-xdist`_ (for parallel unit tests)
-  `mypy`_ (for type check)
-  `sphinx`_ (for document generation)
-  `ruff`_ (for code style check)

.. _`Python 3.12.x`: https://www.python.org/
.. _`NumPy`: https://numpy.org/
.. _`SciPy`: https://scipy.org/
.. _`Matplotlib`: https://matplotlib.org/
.. _`cvxpy`: https://www.cvxpy.org/
.. _`pytest`: https://docs.pytest.org/en/latest/
.. _`pytest-xdist`: https://github.com/pytest-dev/pytest-xdist
.. _`mypy`: https://mypy-lang.org/
.. _`sphinx`: https://www.sphinx-doc.org/en/master/index.html
.. _`ruff`: https://github.com/astral-sh/ruff

