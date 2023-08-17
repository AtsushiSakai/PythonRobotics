
=======================================
🚀 Getting Started with PythonRobotics
=======================================

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Welcome to the PythonRobotics Documentation 👋
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Welcome to PythonRobotics's official documentation, an open source collection of robotics algorithms designed to easily understand the basic ideas behind each algorithm for beginners. 
Let's dive in! 🎉 

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Introduction to PythonRobotics:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PythonRobotics isn't just about cutting down your effort, it's about understanding and learning. 
It's designed to be a useful resource for budding robotics enthusiasts looking to grasp the fundamental concepts behind each algorithm. 
With its intuitive animations, PythonRobotics simplifies complex concepts, making simulations easily understandable in real-world applications.

Based on the principles of collaboration and knowledge sharing, PythonRobotics is an open source software (OSS).
This project includes a selection of practical algorithms that are widely adopted across academic and industrial domains.

Each algorithm in PythonRobotics is coded in Python 3, one of the most popular and powerful programming languages in the world. 
We invite you to explore, learn, and contribute to our open source community, and look forward to being part of your exciting journey in the world of robotics.


~~~~~~~~~~~
MIT License
~~~~~~~~~~~

PythonRobotics is licensed under the MIT License, one of the most permissive open-source licenses. This means:

- **Freedom:** You can use, modify, and distribute the software in any way you choose as long as you include the original copyright and license notice.

.. role:: 5px

- **No Warranty:** The software is provided "as-is". The authors are not responsible for any harm or challenges arising from its use.

.. role:: 5px

- **Compatibility:** The MIT License is harmonious with many other licenses, making integration with other projects more straightforward.

Please refer to the official `MIT License documentation <https://opensource.org/license/mit/>`_ for detailed information.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Minimal Dependencies Philosophy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

PythonRobotics believes in simplicity. A hallmark of our project is its minimal dependencies, ensuring ease of use and broad adaptability. Fewer external dependencies mean:

- **Ease of Execution:** Run code samples effortlessly without extensive setups.

.. role:: 5px

- **Portability:** PythonRobotics codes can be translated to other programming languages like C++ or Java, ensuring versatility in application.



-------------
Dependencies:
-------------

- **Python 3.11.x** : The primary programming language in which the project is written. Refer to the official `Python documentation <https://www.python.org/>`_ for detailed information.

.. role:: 5px

- **NumPy** : A library for numerical operations in Python. It aids in performing matrix and vector operations. Detailed information can be found in the official `NumPy documentation. <https://numpy.org/>`_

.. role:: 5px

- **SciPy** : A scientific computing library that complements NumPy, offering additional functionalities and optimization techniques. More details are available in the official `SciPy documentation. <https://scipy.org/>`_

.. role:: 5px

- **Matplotlib** : A powerful visualization and plotting library in Python. It's crucial for visual simulations and graphical representations of the algorithms. Check the official `Matplotlib documentation <https://matplotlib.org/>`_ for a deeper dive.    

.. role:: 5px

- **Cvxpy** :  Used for convex optimization problems, allowing the project to solve certain algorithmic challenges more efficiently. For a comprehensive guide, see the official `Cvxpy documentation. <https://www.cvxpy.org/>`_

--------------------------
Development Dependencies :
--------------------------

-  **pytest** : A popular testing tool in Python, allowing developers to write simple and scalable test cases. Detailed information can be found in the official `pytest documentation. <https://docs.pytest.org/en/latest/>`_

.. role:: 5px

-  **pytest-xdist** : An extension of pytest that facilitates parallel execution of tests, speeding up the testing process. Check out the official `pytest-xdist github repository <https://github.com/pytest-dev/pytest-xdist>`_ for more details.

.. role:: 5px

-  **mypy** : A static type checker for Python, ensuring type consistency throughout the codebase. For a deeper understanding, refer to the official `mypy documentation <https://mypy-lang.org/>`_

.. role:: 5px

-  **sphinx** : A tool that produces elegant and robust documentation for Python projects. Dive deeper by visiting the official `Sphinx documentation <https://www.sphinx-doc.org/en/master/index.html>`_

.. role:: 5px

-  **ruff** : A utility for checking code style adherence. Check out the official `ruff github repository <https://github.com/charliermarsh/ruff>`_ for more details.



For those keen on a thorough understanding of the PythonRobotics project and its underlying algorithms, please refer to the published paper titled, "PythonRobotics: a Python code collection of robotics algorithms," available on `arXiv. <https://arxiv.org/abs/1808.10703>`_ 
This paper offers a comprehensive overview and serves as an excellent resource for both newcomers and experts.



-----------------------------------------------------
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