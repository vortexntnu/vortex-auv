from setuptools import find_packages, setup

package_name = 'dock_action_servers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'go_to_dock_server = dock_action_servers.go_to_dock_server:main',
            'find_dock_server = dock_action_servers.find_dock_server:main',
            'dock_server = dock_action_servers.dock_server:main',
            'return_home_server = dock_action_servers.return_home_server:main',
            'go_down_server = dock_action_servers.go_down_server:main',
            'go_over_dock_server = dock_action_servers.go_over_dock_server:main',
        ],
    },
)
