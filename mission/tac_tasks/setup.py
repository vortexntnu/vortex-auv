#!/usr/bin/python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['libdockingfsm', 'libpipelinefsm'],
    author='Lasse Moen Guttormsen, Aleksander Klund, Benjaminas Visockis',
    author_email=['aleksander.klu@hotmail.com', 'benjamin.visockis@gmail.com'],
    package_dir={'': 'src'})

setup(**d)
