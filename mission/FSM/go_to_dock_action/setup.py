from setuptools import find_packages, setup

package_name = 'go_to_dock_action'

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
            'go_to_dock_server = go_to_dock_action.go_to_dock_server:main',
            'go_to_dock_client = go_to_dock_action.go_to_dock_client:main',
        ],
    },
)
