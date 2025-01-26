import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pipeline_action_servers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vetlets',
    maintainer_email='vetlets@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'find_pipeline_start_server = pipeline_action_servers.find_pipeline_start_server:main',
            'go_to_pipeline_server = pipeline_action_servers.go_to_pipeline_server:main',
            'return_home_server = pipeline_action_servers.return_home_server:main',
            'fsm_state_node = pipeline_action_servers.fsm_state_node:main',
        ],
    },
)
