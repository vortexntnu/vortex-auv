from setuptools import setup

package_name = 'vortex_stonefish_test_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jorgen Fjermedal',
    maintainer_email='jorgen.fjermedal@hotmail.com',
    description='Shared utilities for simulation-based tests',
    license='MIT',
)
