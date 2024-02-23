from setuptools import find_packages, setup
import glob

package_name = 'thruster_interface_auv'

# Dynamically include all CSV files in the resource directory
csv_files = glob.glob('resources/*.csv')

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all CSV files
        ('share/' + package_name + '/resources', csv_files),
        # Include launch file
        ('share/' + package_name + '/launch', ['launch/thruster_interface_auv.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vortex',
    maintainer_email='vortex.git@vortexntnu.no',
    description='Thruster interface to controll thrusters through PCA9685 Module',
    license='TMIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_interface_auv_node = thruster_interface_auv.thruster_interface_auv_node:main'
        ],
    },
)
