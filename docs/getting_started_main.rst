.. _`getting started`:

Getting Started
===============

==============================================
Welcome to the PythonRobotics Documentation 👋
==============================================

Welcome to PythonRobotics's official documentation, an open source collection of robotics algorithms designed to easily understand the basic ideas behind each algorithm for beginners. 


Introduction to PythonRobotics
-------------------------------

PythonRobotics isn't just about cutting down your effort, it's about understanding and learning. 
It's designed to be a useful resource for budding robotics enthusiasts looking to grasp the fundamental concepts behind each algorithm.
With its intuitive animations, PythonRobotics simplifies complex concepts, making simulations easily understandable in real-world applications.


Based on the principles of collaboration and knowledge sharing, PythonRobotics is an open source software (OSS). 
This project includes a selection of practical algorithms that are widely adopted across academic and industrial domains.


Each algorithm in PythonRobotics is coded in Python 3, one of the most popular and powerful programming languages in the world. 
PythonRobotics is your go-to solution for effective and efficient robotic algorithm development, whether you're an amateur or an experienced professional.
We invite you to explore, learn, and contribute to our open source community, and look forward to being part of your exciting journey in the world of robotics.


See this paper for more details:

- PythonRobotics: a Python code collection of robotics algorithms: https://arxiv.org/abs/1808.10703

.. _`Requirements`:

Requirements
-------------

-  `Python 3.11.x`_
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

.. _`Python 3.11.x`: https://www.python.org/
.. _`NumPy`: https://numpy.org/
.. _`SciPy`: https://scipy.org/
.. _`Matplotlib`: https://matplotlib.org/
.. _`cvxpy`: https://www.cvxpy.org/
.. _`pytest`: https://docs.pytest.org/en/latest/
.. _`pytest-xdist`: https://github.com/pytest-dev/pytest-xdist
.. _`mypy`: https://mypy-lang.org/
.. _`sphinx`: https://www.sphinx-doc.org/en/master/index.html
.. _`ruff`: https://github.com/charliermarsh/ruff


To use the PythonRobotics,you can follow these steps:
-----------------------------------------------------

1. Clone the repository:

.. code-block::

     git clone https://github.com/AtsushiSakai/PythonRobotics.git



2. Navigate to the cloned directory:

.. code-block::

     cd PythonRobotics


3. Install the required libraries:
      
    - If you are using conda, create a new environment from the provided YAML file:
            
        .. code-block::

            conda env create -f requirements/environment.yml

    - If you are using pip, install the required packages from the provided requirements.txt file:
            
        .. code-block::
                
            pip install -r requirements/requirements.txt
       
4. After installing the required libraries, you can execute the Python scripts in each directory according to your needs. Explore the different directories to find the specific scripts for various robotics applications.
 
.. role:: 7px

5. If you find the project useful and like it, consider giving it a star on GitHub ⭐. This is done by clicking the star button on the repository page 🤗 . 


Ensure that Python is installed on your system before proceeding. If you encounter any problems while installing or using the project, please refer to the project's documentation or open an issue on its GitHub repository. 