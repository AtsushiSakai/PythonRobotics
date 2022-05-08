How To Contribute
=================

This document describes how to contribute this project.

Adding a new algorithm example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is a step by step manual to add a new algorithm example.

Step 1: Choose an algorithm to implement
-----------------------------------------

Before choosing an algorithm, please check the :ref:`getting started` doc to
understand this project's philosophy and setup your development environment.

If an algorithm is widely used and successful, let's create an issue to
propose it for our community.

If some people agree by thumbs up or posting positive comments, let go to next step.

It is OK to just create an issue to propose adding an algorithm, someone might implement it.

In that case, please share any papers or documentations to implement it.


Step 2: Implement the algorithm with matplotlib based animation
----------------------------------------------------------------

When you implement an algorithm, please keep the following items in mind.

1. Use only Python. Other language code is not acceptable.

2. This project only accept codes for python 3.9 or higher.

3. Use matplotlib based animation to show how the algorithm works.

4. Only use current :ref:`Requirements` libraries, not adding new dependencies.

5. Keep simple your code. The main goal is to make it easy for users to understand the algorithm, not for practical usage.


Step 3: Add a unittest
----------------------
If you add a new algorithm sample code, please add a unit test file under `tests dir`_.

This sample test code might help you : `test_a_star.py`_.

At the least, try to run the example code without animation in the unit test.

If you want to run the test suites locally, you can use the `runtests.sh` script by just executing it.

The `test_diff_codestyle.py`_ check code style for your PR's codes.


.. _`how to write doc`:

Step 4: Write a document about the algorithm
----------------------------------------------
Please add a document to describe the algorithm details, mathematical backgrounds and show graphs and animation gif.

This project is using `Sphinx`_ as a document builder, all documentations are written by `reStructuredText`_.

You can add a new rst file under the subdirectory in `doc modules dir`_ and the top rst file can include it.

Please check other documents for details.

You can build the doc locally based on `doc README`_.

Note that the `reStructuredText`_ based doc should only focus on the mathematics and the algorithm of the example.

Documentations related codes should be in the python script as the header comments of the script or docstrings of each function.


.. _`submit a pull request`:

Step 5: Submit a pull request and fix codes based on review
------------------------------------------------------------

Let's submit a pull request when your code, test, and doc are ready.

At first, please fix all CI errors before code review.

You can check your PR doc from the CI panel.

After the "ci/circleci: build_doc" CI is succeeded,
you can access you PR doc with clicking the [Details] of the "ci/circleci: build_doc artifact" CI.

.. image:: /_static/img/doc_ci.png

After that, I will start the review.

Note that this is my hobby project; I appreciate your patience during the review process.


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

Supporting this project
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Supporting this project financially is also a great contribution!!.

If you or your company would like to support this project, please consider:

- `Sponsor @AtsushiSakai on GitHub Sponsors`_

- `Become a backer or sponsor on Patreon`_

- `One-time donation via PayPal`_

If you would like to support us in some other way, please contact with creating an issue.

Sponsors
---------

1. `JetBrains`_ : They are providing a free license of their IDEs for this OSS development.


.. _`Python Robotics Docs`: https://atsushisakai.github.io/PythonRobotics
.. _`bug labeled issues`: https://github.com/AtsushiSakai/PythonRobotics/issues?q=is%3Aissue+is%3Aopen+label%3Abug
.. _`tests dir`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/tests
.. _`test_a_star.py`: https://github.com/AtsushiSakai/PythonRobotics/blob/master/tests/test_a_star.py
.. _`Sphinx`: https://www.sphinx-doc.org/
.. _`reStructuredText`: https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html
.. _`doc modules dir`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/docs/modules
.. _`doc README`: https://github.com/AtsushiSakai/PythonRobotics/blob/master/docs/README.md
.. _`test_diff_codestyle.py`: https://github.com/AtsushiSakai/PythonRobotics/blob/master/tests/test_diff_codestyle.py
.. _`JetBrains`: https://www.jetbrains.com/
.. _`Sponsor @AtsushiSakai on GitHub Sponsors`: https://github.com/sponsors/AtsushiSakai
.. _`Become a backer or sponsor on Patreon`: https://www.patreon.com/myenigma
.. _`One-time donation via PayPal`: https://www.paypal.me/myenigmapay/


