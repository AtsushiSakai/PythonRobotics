#!/usr/bin/env python
"""
Test vector module

author - Jev Kuznetsov
"""


from PythonRobotics.vectors import Vector
import numpy as np
import math

# test tuples
ta = (1, 2)
tb = (3, 4)

a = Vector(ta)
b = Vector(tb)
c = Vector(1, 0)
d = Vector(0, 1)
e = Vector(1, 1)


def test_creation():
    an = np.array(ta)
    v = Vector(ta)
    assert (an == v.__array__()).all()

    v = Vector(an)
    assert (an == v.__array__()).all()


def test_add():
    assert a + b == Vector(4, 6)


def test_cross():
    assert np.cross(a, b) == a.cross(b)
    assert np.cross(c, d) == c.cross(d)


def test_sub():
    assert a - b == Vector(-2, -2)


def test_inner():
    assert (a.inner(b) == np.inner(ta, tb)).all()


def test_mul():
    assert a * 3 == Vector(3, 6)

def test_div():
    a = Vector(1,2)
    assert a / 2 == Vector(0.5,1.)

def test_neg():
    assert -a == Vector(-1,-2)

def test_length():
    assert abs(c) == 1
    assert abs(d) == 1
    assert abs(e) == np.sqrt(2)

def test_angle():

    assert c.angle(c) == 0
    assert d.angle(c) == np.pi/2
    assert d.angle(e) == np.pi/4
    assert -d.angle(c) == -np.pi/2

def test_from_polar():

    assert Vector.from_polar(1,0) == Vector(1,0)
