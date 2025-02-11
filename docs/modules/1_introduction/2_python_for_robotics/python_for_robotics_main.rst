Python for Robotics
----------------------

A programing language, Python is used for this `PythonRobotics` project
to achieve the purposes of this project described in the :ref:`What is PythonRobotics?`.

This section explains the Python itself and features for science computing and robotics.

Python for general-purpose programming
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`Python <https://www.python.org/>`_ is an general-purpose programming language developed by
`Guido van Rossum <https://en.wikipedia.org/wiki/Guido_van_Rossum>`_ from the late 1980s.

It features as follows:

#. High-level
#. Interpreted
#. Dynamic type system (also type annotation is supported)
#. Emphasizes code readability
#. Rapid prototyping
#. Batteries included
#. Interoperability for C and Fortran

Due to these features, Python is one of the most popular programming language
for educational purposes for programming beginners.

Python for Scientific Computing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Python itself was not designed for scientific computing.
However, scientists quickly recognized its strengths.
For example,

#. High-level and interpreted features enable scientists to focus on their problems without dealing with low-level programming tasks like memory management.
#. Code readability, rapid prototyping, and batteries included features enables scientists who are not professional programmers, to solve their problems easily.
#. The interoperability to wrap C and Fortran libraries enables scientists to access already existed powerful and optimized scientific computing libraries.

To address the more needs of scientific computing, many fundamental scientific computation libraries have been developed based on the upper features.

- `NumPy <https://numpy.org/>`_ is the fundamental package for scientific computing with Python.
- `SciPy <https://www.scipy.org/>`_ is a library that builds on NumPy and provides a large number of functions that operate on NumPy arrays and are useful for different types of scientific and engineering applications.
- `Matplotlib <https://matplotlib.org/>`_ is a plotting library for the Python programming language and its numerical mathematics extension NumPy.
- `Pandas <https://pandas.pydata.org/>`_ is a fast, powerful, flexible, and easy-to-use open-source data analysis and data manipulation library built on top of NumPy.
- `SymPy <https://www.sympy.org/>`_ is a Python library for symbolic mathematics.
- `CVXPy <https://www.cvxpy.org/>`_ is a Python-embedded modeling language for convex optimization problems.

Also, more domain-specific libraries have been developed based on these fundamental libraries:

- `Scikit-learn <https://scikit-learn.org/stable/>`_ is a free software machine learning library for the Python programming language.
- `Scikit-image <https://scikit-image.org/>`_ is a collection of algorithms for image processing.
- `Networkx <https://networkx.org/>`_ is a package for the creation, manipulation for complex networks.
- `SunPy <https://sunpy.org/>`_ is a community-developed free and open-source software package for solar physics.
- `Astropy <https://www.astropy.org/>`_ is a community-developed free and open-source software package for astronomy.

Currently, Python is one of the most popular programming languages for scientific computing.

Python for Robotics
^^^^^^^^^^^^^^^^^^^^

Python has become an increasingly popular language in robotics.

These are advantages of Python for Robotics:

Simplicity and Readability
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Python's syntax is clear and concise, making it easier to learn and write code.
This is crucial in robotics where complex algorithms and control logic are involved.


Extensive libraries for scientific computation.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Scientific computation routine are fundamental for robotics.
For example:

- Matrix operation is needed for rigid body transformation, state estimation, and model based control.
- Optimization is needed for optimization based SLAM, optimal path planning, and optimal control.
- Visualization is needed for robot teleoperation, debugging, and simulation.

ROS supports Python
~~~~~~~~~~~~~~~~~~~~~~~~~~~
`ROS`_ (Robot Operating System) is an open-source and widely used framework for robotics development.
It is designed to help developping complicated robotic applications.
ROS provides essential tools, libraries, and drivers to simplify robot programming and integration.

Key Features of ROS:

- Modular Architecture – Uses a node-based system where different components (nodes) communicate via messages.
- Hardware Abstraction – Supports various robots, sensors, and actuators, making development more flexible.
- Powerful Communication System – Uses topics, services, and actions for efficient data exchange between components.
- Rich Ecosystem – Offers many pre-built packages for navigation, perception, and manipulation.
- Multi-language Support – Primarily uses Python and C++, but also supports other languages.
- Simulation & Visualization – Tools like Gazebo (for simulation) and RViz (for visualization) aid in development and testing.
- Scalability & Community Support – Widely used in academia and industry, with a large open-source community.

ROS has strong Python support (`rospy`_ for ROS1 and `rclpy`_ for ROS2).
This allows developers to easily create nodes, manage communication between
different parts of a robot system, and utilize various ROS tools.

.. _`ROS`: https://www.ros.org/
.. _`rospy`: http://wiki.ros.org/rospy
.. _`rclpy`: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

Cross-Platform Compatibility
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Python code can run on various operating systems (Windows, macOS, Linux), providing flexibility in choosing hardware platforms for robotics projects.

Large Community and Support
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Python has a vast and active community, offering ample resources, tutorials, and support for developers. This is invaluable when tackling challenges in robotics development.

Situations which Python is NOT suitable for Robotics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We explained the advantages of Python for robotics.
However, Python is not always the best choice for robotics development.

These are situations where Python is NOT suitable for robotics:

High-speed real-time control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Python is an interpreted language, which means it is slower than compiled languages like C++.
This can be a disadvantage when real-time control is required,
such as in high-speed motion control or safety-critical systems.

So, for these applications, we recommend to understand the each algorithm you
needed using this project and implement it in other suitable languages like C++.

Resource-constrained systems
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Python is a high-level language that requires more memory and processing power
compared to low-level languages.
So, it is difficult to run Python on resource-constrained systems like
microcontrollers or embedded devices.
In such cases, C or C++ is more suitable for these applications.
