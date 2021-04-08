#!/usr/bin/env python
"""
Test turtlebot simulator

author - Jev Kuznetsov
"""

import conftest
from PythonRobotics.PythonRobotics import turtlebot as m


def test_1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
