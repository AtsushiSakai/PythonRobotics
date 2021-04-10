#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from setuptools import setup, find_packages


setup(
      name='PythonRobotics',
      version = '1.0.1',
      description="Python implementation of robotics algoritms",
      author= "Atsushi Sakai et al.",
      packages=find_packages(include=['PythonRobotics']),
      url = "https://github.com/AtsushiSakai/PythonRobotics",
      install_requires=[],
      license='MIT'
      )