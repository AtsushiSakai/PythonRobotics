#!/usr/bin/env python
"""
Test vector module

author - Jev Kuznetsov
"""


from PythonRobotics.vectors import Vector2d
import numpy as np

# test tuples
ta = (1, 2)
tb = (3, 4)

a = Vector2d(ta)
b = Vector2d(tb)
c = Vector2d(1, 0)
d = Vector2d(0, 1)
e = Vector2d(1, 1)


def test_creation():
    an = np.array(ta)
    v = Vector2d(ta)
    assert (an == v.__array__()).all()

    v = Vector2d(an)
    assert (an == v.__array__()).all()


def test_add():
    assert a + b == Vector2d(4, 6)


def test_cross():
    assert np.cross(a, b) == a.cross(b)
    assert np.cross(c, d) == c.cross(d)


def test_sub():
    assert a - b == Vector2d(-2, -2)


def test_inner():
    assert (a.inner(b) == np.inner(ta, tb)).all()


def test_mul():
    assert a * 3 == Vector2d(3, 6)

def test_div():
    a = Vector2d(1,2)
    assert a / 2 == Vector2d(0.5,1.)

def test_neg():
    assert -a == Vector2d(-1,-2)

def test_length():
    assert abs(c) == 1
    assert abs(d) == 1
    assert abs(e) == np.sqrt(2)

def test_angle():

    assert c.angle(c) == 0
    assert d.angle(c) == np.pi/2
    assert d.angle(e) == np.pi/4
    assert -d.angle(c) == -np.pi/2
