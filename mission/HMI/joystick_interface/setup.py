#!/usr/bin/python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# setup(
#     version="1.0.0",
#     description="Library for the joystick interface",
#     author="Benjaminas Visockis",
#     author_email="benjamin.visockis@gmail.com",
#     packages=['libjoystick'],
#     package_dir={'': 'src'}
# )

d = generate_distutils_setup(
    packages=['libjoystick'],
    package_dir={'': 'src'}
)

setup(**d)