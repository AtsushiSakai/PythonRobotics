#!/usr/bin/env python
"""
Test turtle graphics

author - Jev Kuznetsov
"""

import conftest
from PythonRobotics import graphics as m


def test_1():
    m.TESTING = True
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
