#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Fetch configuration arguments from package.xml
d = generate_distutils_setup(
    packages=['motor_lib', 'sense_lib'],
    package_dir={'': '.'}
)

setup(**d)