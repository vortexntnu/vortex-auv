#!/usr/bin/python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['task_manager_defines', 'task_manager_client'],
    author='Ronja Kræmer, Benjaminas Visockis',
    author_email=['ronjak@stud.ntnu.no', 'benjamin.visockis@gmail.com'],
    package_dir={'': 'src'})

setup(**d)
