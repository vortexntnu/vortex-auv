from setuptools import setup

package_name = 'test_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jorgen Fjermedal',
    maintainer_email='jorgen.fjermedal@hotmail.com',
    description='Shared utilities for simulation-based tests',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
