#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['tiago_move_interface'],
     package_dir={'': 'scripts'}
)

setup(**setup_args)