from setuptools import setup
import os
from glob import glob

package_name = 'auv_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'matplotlib',
        'PyQt6',
    ],
    zip_safe=True,
    maintainer='sondre',
    maintainer_email='sondre95556888@gmail.com',
    description='AUV GUI for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auv_gui_node = auv_gui.auv_gui:main'
        ],
    },
    scripts=[],  # Add this line
    python_requires='>=3.8',  # Add this line
)