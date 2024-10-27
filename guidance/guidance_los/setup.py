from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'guidance_los'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Updated launch files inclusion
        (os.path.join('share', package_name,
                      'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='badawi',
    maintainer_email='badawi@todo.todo',
    description='Package for guidance node implementing PID control.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'guidance = guidance_los.guidance_los:main',
            'guidance_test = guidance_los.guidance_test:main',
            'odom_formatter = guidance_los.odom_formatter:main'
        ],
    },
)
