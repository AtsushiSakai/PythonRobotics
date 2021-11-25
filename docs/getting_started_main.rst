.. _`getting started`:

Getting Started
===============

What is this?
-------------

This is an Open Source Software (OSS) project: PythonRobotics, which is a Python code collection of robotics algorithms.

The focus of the project is on autonomous navigation, and the goal is for beginners in robotics to understand the basic ideas behind each algorithm.

In this project, the algorithms which are practical and widely used in both academia and industry are selected.

Each sample code is written in Python3 and only depends on some standard modules for readability and ease of use. 

It includes intuitive animations to understand the behavior of the simulation.


See this paper for more details:

- PythonRobotics: a Python code collection of robotics algorithms: https://arxiv.org/abs/1808.10703

.. _`Requirements`:

Requirements
-------------

-  `Python 3.9.x`_
-  `NumPy`_
-  `SciPy`_
-  `Matplotlib`_
-  `pandas`_
-  `cvxpy`_

For development:

-  pytest (for unit tests)
-  pytest-xdist (for parallel unit tests)
-  mypy (for type check)
-  sphinx (for document generation)
-  pycodestyle (for code style check)

.. _`Python 3.9.x`: https://www.python.org/
.. _`NumPy`: https://numpy.org/
.. _`SciPy`: https://scipy.org/
.. _`Matplotlib`: https://matplotlib.org/
.. _`pandas`: https://pandas.pydata.org/
.. _`cvxpy`: https://www.cvxpy.org/


How to use
----------

1. Clone this repo and go into dir.

.. code-block::

    >$ git clone https://github.com/AtsushiSakai/PythonRobotics.git

    >$ cd PythonRobotics


2. Install the required libraries.

using conda :

.. code-block::

    >$ conda env create -f environment.yml

using pip :

.. code-block::

    >$ pip install -r requirements.txt


3. Execute python script in each directory.

4. Add star to this repo if you like it 😃.

