from setuptools import setup

package_name = 'auv_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sondre',
    maintainer_email='sondre@example.com',
    description='AUV GUI for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auv-gui = auv_gui.auv_gui:main',
        ],
    },
)
