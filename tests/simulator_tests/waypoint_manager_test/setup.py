from setuptools import setup, find_packages

package_name = 'waypoint_manager_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jorgen Fjermedal',
    maintainer_email='jorgen.fjermedal@hotmail.com',
    description='waypoint manager test',
    license='MIT',
)
