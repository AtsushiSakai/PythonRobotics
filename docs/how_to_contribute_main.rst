How to contribute
=================

This document describes how to contribute this project.

Adding a new algorithm example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is a step by step manual to add a new algorithm example.

Step 1: Choose an algorithm to implement
-----------------------------------------

Before choosing an algorithm, please check the :ref:`getting started` doc to
understand this project's philosophy.

If an algorithm is widely used and successful, let's create an issue to
propose it for our community.

If some people agree by thumbs up or posting positive comments, let go to next step.

It is OK to just create an issue to propose adding an algorithm, someone might implement it.

In that case, please share any papers or documentations to implement it.


Step 2: Implement the algorithm with matplotlib based animation
----------------------------------------------------------------

This project only accept codes for python 3.9 or higher.

Step 3: Add a unittest
----------------------
If you add a new algorithm sample code, please add a unit test file under `tests` dir.

This sample test code might help you : https://github.com/AtsushiSakai/PythonRobotics/blob/master/tests/test_a_star.py

.. _`how to write doc`:

Step 4: Write a document about the algorithm
----------------------------------------------


.. _`submit a pull request`:

Step 5: Submit a pull request and fix codes based on review
------------------------------------------------------------

Please fix all issues on CI (All CI should be green), before code review.

Reporting and fixing a defect
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Reporting and fixing a defect is also great contribution.

When you report an issue, please provide these information:

- A clear and concise description of what the bug is.
- A clear and concise description of what you expected to happen.
- Screenshots to help explain your problem if applicable.
- OS version
- Python version
- Each library versions

If you want to fix any bug, you can find reported issues in `bug labeled issues`_.

If you fix a bug of existing codes, please add a test function
in the test code to show the issue was solved.

This doc `submit a pull request`_ can be helpful to submit a pull request.

Adding missed documentations for existing examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Adding the missed documentations for existing examples is also great contribution.

If you check the `Python Robotics Docs`_, you can notice that some of the examples
only have a simulation gif or short overview descriptions,
but no detailed algorithm or mathematical description.

This doc `how to write doc`_ can be helpful to write documents.

.. _`Python Robotics Docs`: https://pythonrobotics.readthedocs.io/en/latest/
.. _`bug labeled issues`: https://github.com/AtsushiSakai/PythonRobotics/issues?q=is%3Aissue+is%3Aopen+label%3Abug

