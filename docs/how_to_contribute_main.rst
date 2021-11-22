How to contribute
=================

This document describes how to contribute this project.

Adding a new algorithm example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Step 1: Choose an algorithm to implement
-----------------------------------------
This project only accept codes for python 3.9 or higher.

Step 2: Implement the algorithm with matplotlib based animation
----------------------------------------------------------------

Step 3: Add a unittest
----------------------
If you add a new algorithm sample code, please add a unit test file under `tests` dir.

This sample test code might help you : https://github.com/AtsushiSakai/PythonRobotics/blob/master/tests/test_a_star.py

.. _`how to write doc`:

Step 4: Write a document about the algorithm
----------------------------------------------


Step 5: Submit a pull request and fix codes based on review
------------------------------------------------------------

Please fix all issues on CI (All CI should be green), before code review.

Reporting and fixing a defect
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Reporting and fixing a defect is also great contribution.

You can find reported issues in `bug labeled issues`_.

If you fix a bug of existing codes, please add a test function
in the test code to show the issue was solved.

Adding missed documentations for existing examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you check the `PythonRoboticsDocs`_, you can notice that some of the examples
only have a simulation gif or short overview descriptions,
but no detailed algorithm or mathematical description.

Adding the missed documentations for existing examples is also great contribution.

This doc `how to write doc`_ can be helpful..

.. _`PythonRoboticsDocs`: https://pythonrobotics.readthedocs.io/en/latest/
.. _`bug labeled issues`: https://github.com/AtsushiSakai/PythonRobotics/issues?q=is%3Aissue+is%3Aopen+label%3Abug

