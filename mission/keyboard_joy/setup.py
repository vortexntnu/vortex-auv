from setuptools import setup
import os
from glob import glob

package_name = 'keyboard_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='kluge7',
    maintainer_email='andreas.svendsrud@vortexntnu.no',
    description='Keyboard teleop node that publishes sensor_msgs/Joy messages',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keyboard_joy_node = keyboard_joy.keyboard_joy_node:main',
        ],
    },
)
