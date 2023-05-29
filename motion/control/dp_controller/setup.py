#!/usr/bin/python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=['dp_client_py'],
                             package_dir={'': 'src'})

setup(**d)
