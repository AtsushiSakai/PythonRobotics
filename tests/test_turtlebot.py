#!/usr/bin/env python
"""
Test turtlebot simulator

to run this test first install the PythonRobotics as development version.
do this by running
>pip install -e .
from the main PythonRobotics directory.
This removes the need for the `conftest` hack ;-).

author - Jev Kuznetsov
"""

from PythonRobotics.PythonRobotics import turtlebot as m


def test_1():
    m.show_animation = False
    m.main()